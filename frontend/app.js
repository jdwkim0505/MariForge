import * as THREE from "three";
import { OrbitControls } from "three/addons/controls/OrbitControls.js";
import { STLLoader } from "three/addons/loaders/STLLoader.js";

const $ = (id) => document.getElementById(id);
const fmt = (v, d = 0) => Number(v).toFixed(d);
let designs = [], current = null, pollTimer = null;

// ---- viewer ---------------------------------------------------------------
const host = $("viewer");
const renderer = new THREE.WebGLRenderer({ antialias: true });
renderer.setPixelRatio(window.devicePixelRatio);
host.appendChild(renderer.domElement);
const scene = new THREE.Scene();
scene.background = new THREE.Color(0x0e1620);
const camera = new THREE.PerspectiveCamera(40, 1, 1, 20000);
camera.up.set(0, 0, 1);
const controls = new OrbitControls(camera, renderer.domElement);
scene.add(new THREE.HemisphereLight(0xdfefff, 0x203040, 1.1));
const sun = new THREE.DirectionalLight(0xffffff, 1.2);
sun.position.set(500, -800, 900);
scene.add(sun);

const boat = new THREE.Group();      // rotated by the heel slider
scene.add(boat);
const water = new THREE.Mesh(
  new THREE.PlaneGeometry(1, 1),
  new THREE.MeshBasicMaterial({ color: 0x2a7fb8, transparent: true, opacity: 0.35, side: THREE.DoubleSide, depthWrite: false }));
scene.add(water);

const MATS = {
  envelope: new THREE.MeshStandardMaterial({ color: 0x9fb3c6, roughness: 0.6, transparent: true, opacity: 0.55 }),
  hull: new THREE.MeshStandardMaterial({ color: 0xd9dee4, roughness: 0.55, metalness: 0.05 }),
  hatch_lid: new THREE.MeshStandardMaterial({ color: 0x3372bf, roughness: 0.5 }),
  rudder: new THREE.MeshStandardMaterial({ color: 0xcc4d33, roughness: 0.5 }),
};

function resize() {
  const w = host.clientWidth, h = host.clientHeight;
  renderer.setSize(w, h);
  camera.aspect = w / h;
  camera.updateProjectionMatrix();
}
new ResizeObserver(resize).observe(host);
(function loop() { requestAnimationFrame(loop); controls.update(); renderer.render(scene, camera); })();

const loader = new STLLoader();
function loadStl(url, mat) {
  return new Promise((ok, bad) => loader.load(url, (g) => { g.computeVertexNormals(); ok(new THREE.Mesh(g, mat)); }, undefined, bad));
}

function frame(d) {
  const L = d.params.L, B = d.overall.beam_mm;
  controls.target.set(L / 2, 0, 0);
  camera.position.set(L * 1.15, -L * 0.95, L * 0.55);
  water.scale.set(L * 1.5, B * 3.2, 1);
  water.position.set(L / 2, 0, 0);
  boat.position.set(0, 0, 0);
}

async function show(d) {
  boat.clear();
  frame(d);
  $("cad_state").textContent = "loading hull preview (the first one after startup can take a minute)...";
  try {
    boat.add(await loadStl(`/api/preview/${d.id}/envelope.stl`, MATS.envelope));
  } catch (e) {
    const r = await fetch(`/api/preview/${d.id}/envelope.stl`).catch(() => null);
    const why = r && !r.ok ? await r.text() : String(e);
    $("cad_state").textContent = "3D preview failed: " + why.slice(0, 300);
  }
  applyHeel();
  watchCad(d);
}

async function watchCad(d) {
  clearTimeout(pollTimer);
  const r = await fetch(`/api/cad/${d.id}/status`);
  const s = await r.json();
  if (current !== d) return;
  if (s.state === "building") {
    $("cad_state").textContent = "building CAD with hardware...";
    pollTimer = setTimeout(() => watchCad(d), 2500);
    return;
  }
  if (s.state === "failed") { $("cad_state").textContent = "CAD build failed: " + s.error; return; }
  const meshes = await Promise.all(["hull", "hatch_lid", "rudder"].map((p) => loadStl(`/api/preview/${d.id}/${p}.stl`, MATS[p])));
  if (current !== d) return;
  boat.clear();
  meshes.forEach((m) => boat.add(m));
  d.cad = s;
  const v = s.verification;
  const cutsOk = v.cuts.every((c) => c.ok);
  $("cad_state").innerHTML =
    `exported STL displaces ${fmt(100 * (1 + v.volume_error), 2)} % of design, floats at ${fmt(v.mesh_hydrostatics.float_z_mm, 2)} mm, ` +
    `${v.cuts.length} cuts ${cutsOk ? "verified" : "<b class='no'>FAILED</b>"}`;
  renderSpec(d);
}

function applyHeel() {
  const deg = Number($("heel").value);
  $("heel_v").textContent = deg + "°";
  boat.rotation.x = THREE.MathUtils.degToRad(deg);
  water.visible = $("show_wl").checked;
  if (current) {
    // equilibrium waterline height at this heel, fixed trim, from the GZ solve
    const gz = current.hydrostatics.gz;
    const i = Math.min(Math.floor(deg / 5), gz.length - 2), f = (deg - gz[i].heel_deg) / 5;
    water.position.z = gz[i].wl_mm + f * (gz[i + 1].wl_mm - gz[i].wl_mm);
  }
}
$("heel").oninput = applyHeel;
$("show_wl").onchange = applyHeel;

// ---- gallery + spec -------------------------------------------------------
function gallery() {
  $("gallery").innerHTML = "";
  designs.forEach((d) => {
    const el = document.createElement("div");
    el.className = "card" + (d === current ? " sel" : "");
    const bars = Object.entries(d.traits).map(([k, v]) =>
      `<div class="bar"><span>${k.replace("_", " ")}</span><i style="width:${10 + 90 * v}%"></i></div>`).join("");
    el.innerHTML = `<h3>${d.label} <span class="badge ${d.feasible ? "ok" : "no"}">${d.feasible ? "ok" : "check"}</span></h3>
      <p>${d.blurb}</p>${bars}`;
    el.onclick = () => select(d);
    $("gallery").appendChild(el);
  });
}

function svgCurve(pts, w, h, xl, yl, markX) {
  const xs = pts.map((p) => p[0]), ys = pts.map((p) => p[1]);
  const x0 = Math.min(...xs), x1 = Math.max(...xs), y0 = Math.min(0, ...ys), y1 = Math.max(...ys);
  const X = (x) => 30 + (x - x0) / (x1 - x0) * (w - 40), Y = (y) => h - 18 - (y - y0) / (y1 - y0 || 1) * (h - 28);
  const path = pts.map((p, i) => (i ? "L" : "M") + X(p[0]).toFixed(1) + "," + Y(p[1]).toFixed(1)).join("");
  const mark = markX != null ? `<line x1="${X(markX)}" x2="${X(markX)}" y1="8" y2="${h - 18}" stroke="#e3b341" stroke-dasharray="3 3"/>` : "";
  return `<svg width="100%" viewBox="0 0 ${w} ${h}">
    <line x1="30" x2="${w - 10}" y1="${Y(0)}" y2="${Y(0)}" stroke="#243447"/>
    ${mark}<path d="${path}" fill="none" stroke="#3fa7d6" stroke-width="2"/>
    <text x="30" y="10">${yl} (max ${fmt(y1, y1 < 10 ? 2 : 0)})</text><text x="${w - 90}" y="${h - 4}">${xl}</text></svg>`;
}

function renderSpec(d) {
  const h = d.hydrostatics, m = d.mass, pr = d.propulsion, r = d.resistance.cruise, p = d.params;
  $("spec_title").textContent = `${d.label}: displacement monohull`;
  const rows = [
    ["Length x beam x height", `${fmt(p.L)} x ${fmt(d.overall.beam_mm)} x ${fmt(d.overall.height_mm)} mm`],
    ["Waterline beam / draft / freeboard", `${fmt(p.B)} / ${fmt(p.T)} / ${fmt(p.F)} mm`],
    ["Displacement", `${fmt(h.displacement_g)} g`],
    ["Printed parts", `${fmt(m.printed_shell_g)} g shell`],
    ["Ballast / payload allowance", `${fmt(m.ballast.mass_g)} g at x = ${fmt(m.ballast.x ?? 0)} mm`],
    ["Cb / Cp / Cm / Cwp", `${fmt(h.Cb, 2)} / ${fmt(h.Cp, 2)} / ${fmt(h.Cm, 2)} / ${fmt(h.Cwp, 2)}`],
    ["KB / BM / KG / GM", `${fmt(h.kb_mm, 1)} / ${fmt(h.bm_mm, 1)} / ${fmt(h.kg_mm, 1)} / <b>${fmt(h.gm_mm, 1)}</b> mm`],
    ["Downflooding angle", `${fmt(h.downflooding_deg)}°`],
    ["Cruise", `${fmt(r.V_mps, 2)} m/s, Fn ${fmt(r.Fn, 2)}`],
    ["Resistance at cruise", `${fmt(r.R_total_N, 3)} N (${fmt(100 * r.R_wave_N / r.R_total_N)} % wave)`],
    ["Hull speed (Fn 0.40)", `${fmt(d.resistance.hull_speed_mps, 2)} m/s`],
    ["Motor / battery", `${pr.motor.name} / ${pr.battery.name}`],
    ["Electric power at cruise", `${fmt(pr.P_electric_W, 1)} W`],
    ["Endurance at cruise", `${fmt(pr.endurance_min)} min`],
    ["Prop diameter", `${fmt(pr.prop_dia_mm)} mm`],
  ];
  const gz = svgCurve(h.gz.map((g) => [g.heel_deg, g.gz_mm]), 300, 110, "heel (deg)", "GZ mm", h.downflooding_deg);
  const rc = svgCurve(d.resistance.curve.map((c) => [c.V_mps, c.R_total_N]), 300, 110, "speed (m/s)", "R N", r.V_mps);
  const checks = d.checks.map((c) => `<li><b class="${c.ok ? "ok" : "no"}">${c.ok ? "✓" : "✗"}</b> ${c.name}: ${c.detail}</li>`).join("");
  let cad = "";
  if (d.cad) {
    const v = d.cad.verification;
    const bad = v.cuts.filter((c) => !c.ok).map((c) => c.name);
    cad = `<div class="section">Exported geometry (proven on the mesh)</div><table>
      <tr><td>Volume at design waterline</td><td>${fmt(100 * v.volume_error, 2)} % error</td></tr>
      <tr><td>Floats at (design z = 0)</td><td>${fmt(v.mesh_hydrostatics.float_z_mm, 2)} mm</td></tr>
      <tr><td>LCB - LCG</td><td>${fmt(v.mesh_hydrostatics.lcb_minus_lcg_mm, 1)} mm</td></tr>
      <tr><td>Hull solid / watertight</td><td>${v.hull_valid ? "valid" : "INVALID"} / ${v.hull_mesh_watertight ? "yes" : "NO"}</td></tr>
      <tr><td>Hardware cuts</td><td>${bad.length ? "failed: " + bad.join(", ") : v.cuts.length + " verified"}</td></tr>
      </table>
      <button id="dl_step">Download STEP</button>
      <button id="dl_stl" class="ghost">Download STL set (split for a ${d.request.bed_mm} mm bed)</button>`;
  }
  $("spec_body").className = "";
  $("spec_body").innerHTML = `<table>${rows.map(([a, b]) => `<tr><td>${a}</td><td>${b}</td></tr>`).join("")}</table>
    <div class="section">Righting arm (fixed trim)</div>${gz}
    <div class="section">Calm-water resistance</div>${rc}
    <div class="section">Checks</div><ul class="checks">${checks}</ul>${cad}
    <p class="muted">${d.resistance.assumptions}; propulsive efficiency ${pr.eta_propulsive} assumed.</p>`;
  if (d.cad) {
    $("dl_step").onclick = () => (location.href = `/api/export/${d.id}/step`);
    $("dl_stl").onclick = () => (location.href = `/api/export/${d.id}/stl`);
  }
}

function select(d) {
  current = d;
  gallery();
  renderSpec(d);
  show(d);
}

$("go").onclick = async () => {
  const ids = ["length_mm", "beam_mm", "height_mm", "cruise_speed_mps", "endurance_min", "bed_mm"];
  const body = { hull_type: $("hull_type").value, mission: $("mission").value };
  ids.forEach((k) => (body[k] = Number($(k).value)));
  $("go").disabled = true;
  $("msg").textContent = "solving five hulls...";
  try {
    const r = await fetch("/api/generate", { method: "POST", headers: { "Content-Type": "application/json" }, body: JSON.stringify(body) });
    if (!r.ok) throw new Error(await r.text());
    designs = (await r.json()).designs;
    $("msg").textContent = `${designs.filter((d) => d.feasible).length} of 5 pass every check.`;
    select(designs.find((d) => d.feasible) || designs[0]);
  } catch (e) {
    $("msg").textContent = "Failed: " + e.message;
  } finally {
    $("go").disabled = false;
  }
};
