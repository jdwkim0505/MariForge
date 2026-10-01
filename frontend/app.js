import * as THREE from "three";
import { OrbitControls } from "three/addons/controls/OrbitControls.js";
import { STLLoader } from "three/addons/loaders/STLLoader.js";
import { toCreasedNormals } from "three/addons/utils/BufferGeometryUtils.js";

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
  // STL has one normal per triangle, which renders every facet. Smooth across
  // gentle angles, keep real edges (deck edge, coaming, transom) crisp.
  return new Promise((ok, bad) => loader.load(url, (g) => ok(new THREE.Mesh(toCreasedNormals(g, THREE.MathUtils.degToRad(30)), mat)), undefined, bad));
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
    boat.add(payloadMeshes(d));
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
  boat.add(payloadMeshes(d));
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

// ---- payload picker ---------------------------------------------------------
const MOUNT = { bay: "in hull", deck: "on lid", through_hull: "through-hull well" };
let customs = [];
fetch("/api/options").then((r) => r.json()).then((o) => {
  $("payload_list").className = "";
  $("payload_list").innerHTML = o.payloads.map((p) =>
    `<div class="prow"><input type="checkbox" id="pl_${p.key}">
      <span>${p.name}<small>${fmt(p.mass_g)} g, ${p.power_W ? fmt(p.power_W, 1) + " W, " : ""}${MOUNT[p.mount]}</small></span>
      <input type="number" id="plq_${p.key}" value="1" min="1" max="6" title="quantity"></div>`).join("");
  $("payload_list").dataset.keys = o.payloads.map((p) => p.key).join(",");
}).catch(() => ($("payload_list").textContent = "payload catalogue unavailable"));

function drawCustoms() {
  $("custom_list").innerHTML = customs.map((c, i) =>
    `<li><span>${c.name}<small>${fmt(c.mass_g)} g, ${c.dims_mm.map((v) => fmt(v)).join(" x ")} mm, ${MOUNT[c.mount]}</small></span>
      <button type="button" data-i="${i}" title="remove">✕</button></li>`).join("");
  $("custom_list").querySelectorAll("button").forEach((b) => (b.onclick = () => { customs.splice(+b.dataset.i, 1); drawCustoms(); }));
}
$("c_add").onclick = () => {
  const dims = $("c_dims").value.split(/[x×, ]+/).map(Number).filter((v) => v > 0);
  if (dims.length !== 3) { $("msg").textContent = "Custom item size needs three numbers, L x W x H in mm."; return; }
  customs.push({ key: "custom", name: $("c_name").value || "Custom payload", mass_g: Number($("c_mass").value),
    dims_mm: dims, power_W: Number($("c_power").value) || 0, mount: $("c_mount").value });
  drawCustoms();
};

function payloadRequest() {
  const keys = ($("payload_list").dataset.keys || "").split(",").filter(Boolean);
  const rows = keys.filter((k) => $("pl_" + k).checked).map((k) => ({ key: k, qty: Number($("plq_" + k).value) || 1 }));
  return rows.concat(customs);
}

// payload boxes and wells, drawn in the boat frame so they heel with it
const PL_MAT = new THREE.MeshStandardMaterial({ color: 0xe3b341, roughness: 0.6, transparent: true, opacity: 0.85 });
const WELL_MAT = new THREE.MeshStandardMaterial({ color: 0x4cc38a, roughness: 0.6, transparent: true, opacity: 0.6 });
function payloadMeshes(d) {
  const g = new THREE.Group();
  (d.layout.payload || []).forEach((it) => {
    if (it.zone === "well") return;
    const [lx, wy, hz] = it.size;
    const m = new THREE.Mesh(new THREE.BoxGeometry(lx, wy, hz), PL_MAT);
    m.position.set(it.x, it.y, it.z_bottom + hz / 2);
    g.add(m);
  });
  (d.layout.wells || []).forEach((w) => {
    const h = w.top_z - w.z_keel;
    const m = new THREE.Mesh(new THREE.CylinderGeometry(w.bore / 2, w.bore / 2, h, 32, 1, true), WELL_MAT);
    m.rotation.x = Math.PI / 2;
    m.position.set(w.x, 0, w.z_keel + h / 2);
    g.add(m);
  });
  return g;
}

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
    ["Ballast", `${fmt(m.ballast.mass_g)} g at x = ${fmt(m.ballast.x ?? 0)} mm`],
    ["Payload carried / spare capacity", `${fmt(d.payload.mass_g)} g / <b>${fmt(d.payload.spare_capacity_g)} g</b>`],
    ["Cb / Cp / Cm / Cwp", `${fmt(h.Cb, 2)} / ${fmt(h.Cp, 2)} / ${fmt(h.Cm, 2)} / ${fmt(h.Cwp, 2)}`],
    ["KB / BM / KG / GM", `${fmt(h.kb_mm, 1)} / ${fmt(h.bm_mm, 1)} / ${fmt(h.kg_mm, 1)} / <b>${fmt(h.gm_mm, 1)}</b> mm`],
    ["Downflooding angle", `${fmt(h.downflooding_deg)}°`],
    ["Cruise", `${fmt(r.V_mps, 2)} m/s, Fn ${fmt(r.Fn, 2)}`],
    ["Resistance at cruise", `${fmt(r.R_total_N, 3)} N (${fmt(100 * r.R_wave_N / r.R_total_N)} % wave)`],
    ["Hull speed (Fn 0.40)", `${fmt(d.resistance.hull_speed_mps, 2)} m/s`],
    ["Motor / battery", `${pr.motor.name} / ${pr.battery.name}`],
    ["Electric power at cruise", `${fmt(pr.P_electric_W, 1)} W${pr.P_payload_W ? ` + ${fmt(pr.P_payload_W, 1)} W payload` : ""}`],
    ["Endurance at cruise", `${fmt(pr.endurance_min)} min`],
    ["Prop diameter", `${fmt(pr.prop_dia_mm)} mm`],
  ];
  const gz = svgCurve(h.gz.map((g) => [g.heel_deg, g.gz_mm]), 300, 110, "heel (deg)", "GZ mm", h.downflooding_deg);
  const rc = svgCurve(d.resistance.curve.map((c) => [c.V_mps, c.R_total_N]), 300, 110, "speed (m/s)", "R N", r.V_mps);
  const sk = d.seakeeping, np = sk.natural_periods_s;
  const sea = `<div class="section">Sea state</div><table>
      <tr><td>Works in waves up to</td><td><span class="big">Hs ${fmt(sk.max_hs_mm)} mm</span></td></tr>
      <tr><td>Limited by</td><td>${sk.limited_by_label}</td></tr>
      <tr><td>WMO sea state</td><td>${sk.wmo_code}, ${sk.wmo_name}</td></tr>
      <tr><td>Natural periods heave / pitch / roll</td><td>${fmt(np.heave, 2)} / ${fmt(np.pitch, 2)} / ${fmt(np.roll, 2)} s</td></tr>
    </table>${svgCurve(sk.rao_head_seas.map((r) => [r.lambda_over_L, r.heave]), 300, 110, "wavelength / L", "heave RAO")}
    <p class="muted">${sk.assumptions}.</p>`;
  const pl = d.payload.items.length ? `<div class="section">Payload</div><table>${d.payload.items.map((it) =>
      `<tr><td>${it.name}</td><td>${fmt(it.mass_g)} g, ${it.zone}${it.zone === "well" ? "" : ` at x = ${fmt(it.x)}`}</td></tr>`).join("")}</table>` : "";
  const bom = d.bom;
  const bomTable = `<div class="section">Parts list (estimated ~$${fmt(bom.total_usd)})</div>
    <details><summary>${bom.rows.length} lines; print ${fmt(bom.printed_g)} g of PLA in ${fmt(bom.print_hours[0])} to ${fmt(bom.print_hours[1])} h</summary>
    <table class="bom">${bom.rows.map((r) => `<tr><td>${r.item}</td><td>${r.note}</td><td>${r.qty_unit ? fmt(r.qty, 2) + " " + r.qty_unit + ", " : r.qty !== 1 ? r.qty + " x, " : ""}${r.priced ? "$" + fmt(r.usd) : "n/a"}</td></tr>`).join("")}</table>
    </details><p class="muted">${bom.note}</p>
    <button id="dl_ap" class="ghost">Download ArduPilot Rover parameters</button>`;
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
    ${sea}${pl}
    <div class="section">Checks</div><ul class="checks">${checks}</ul>${bomTable}${cad}
    <p class="muted">${d.resistance.assumptions}; propulsive efficiency ${pr.eta_propulsive} assumed.</p>`;
  $("dl_ap").onclick = () => (location.href = `/api/export/${d.id}/ardupilot.param`);
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
  const ids = ["length_mm", "beam_mm", "height_mm", "cruise_speed_mps", "endurance_min", "bed_mm", "wave_height_mm"];
  const body = { hull_type: $("hull_type").value, mission: $("mission").value, payloads: payloadRequest() };
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
