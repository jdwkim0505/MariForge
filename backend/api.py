"""HTTP layer. Physics runs in the request; CAD builds run in a background
thread pool so the gallery comes back at once and previews fill in."""
from __future__ import annotations

import os
import threading
from collections import deque
from concurrent.futures import Future
from pathlib import Path

from fastapi import APIRouter, HTTPException
from fastapi.responses import FileResponse, JSONResponse

from .physics.variants import CHARACTERS, MISSIONS, generate_variants
from .schemas import GenerateRequest

EXPORTS = Path(__file__).resolve().parent.parent / "exports"
EXPORTS.mkdir(exist_ok=True)

router = APIRouter(prefix="/api")
_DESIGNS: dict[str, dict] = {}
_BUILDS: dict[str, Future] = {}
_EXPORTED: dict[str, Future] = {}
_PARTS: dict[str, dict] = {}
_OCC = threading.Lock()          # one OCCT job at a time: builds and exports share it
_LOCK = threading.Lock()
_QUEUE: deque[str] = deque()
_WAKE = threading.Condition(_LOCK)


def _build(design_id: str) -> dict:
    """Parts + verification + preview meshes (what the viewer needs)."""
    from .cad.exporters import export_stl
    from .cad.hull import build_parts
    from .cad.verify import verify_design

    d = _DESIGNS[design_id]
    with _OCC:
        parts = build_parts(d)
        ver = verify_design(d, parts)
    out = {"report": parts["_report"], "verification": ver}
    base = EXPORTS / design_id
    base.mkdir(exist_ok=True)
    with _OCC:
        for k in ("hull", "hatch_lid", "rudder"):
            export_stl(parts[k], base / f"{k}.stl", 0.2, 0.3)
    d["cad"] = out
    _PARTS[design_id] = parts
    return out


def _export_files(design_id: str) -> None:
    from .cad.exporters import export_step, export_stl_zip
    d = _DESIGNS[design_id]
    _schedule(design_id, urgent=True).result()
    base = EXPORTS / design_id
    with _OCC:
        export_step(_PARTS[design_id], base / "mariforge.step")
        export_stl_zip(d, _PARTS[design_id], base / "mariforge_stl.zip", d["cad"]["verification"],
                       d["request"].get("bed_mm", 250))


def _worker() -> None:
    # One CAD worker: OCCT booleans are CPU-bound and gain nothing from threads.
    while True:
        with _WAKE:
            while not _QUEUE:
                _WAKE.wait()
            job = _QUEUE.popleft()
        fut = _BUILDS[job]
        if fut.done():
            continue
        try:
            fut.set_result(_build(job))
        except Exception as e:  # reported by /cad/{id}/status
            fut.set_exception(e)


threading.Thread(target=_worker, daemon=True).start()


def _schedule(design_id: str, urgent: bool = False) -> Future:
    """Queue a build; `urgent` (the design on screen) jumps to the front."""
    with _WAKE:
        fut = _BUILDS.get(design_id)
        if fut is None:
            fut = _BUILDS[design_id] = Future()
            _QUEUE.append(design_id)
        if urgent and not fut.done() and design_id in _QUEUE:
            _QUEUE.remove(design_id)
            _QUEUE.appendleft(design_id)
        _WAKE.notify()
        return fut


@router.get("/options")
def options():
    return {"hull_types": [{"key": "displacement_monohull", "label": "Displacement monohull"}],
            "missions": [{"key": k, **v} for k, v in MISSIONS.items()],
            "characters": [{"key": c["key"], "label": c["label"], "blurb": c["blurb"]} for c in CHARACTERS]}


@router.post("/generate")
def generate(req: GenerateRequest):
    designs = generate_variants(req.model_dump())
    for d in designs:
        _DESIGNS[d["id"]] = d
    # warm the CAD for the feasible designs in the background, best first
    if os.environ.get("MARIFORGE_WARM", "1") != "0":
        for d in sorted(designs, key=lambda d: not d["feasible"]):
            _schedule(d["id"])
    return {"designs": designs}


def _run_export(design_id: str, fut: Future) -> None:
    try:
        _export_files(design_id)
        fut.set_result(True)
    except Exception as e:
        fut.set_exception(e)


@router.get("/design/{design_id}")
def design(design_id: str):
    if design_id not in _DESIGNS:
        raise HTTPException(404)
    return _DESIGNS[design_id]


@router.get("/cad/{design_id}/status")
def cad_status(design_id: str):
    if design_id not in _DESIGNS:
        raise HTTPException(404)
    f = _schedule(design_id, urgent=True)
    if not f.done():
        return {"state": "building"}
    if f.exception():
        return JSONResponse({"state": "failed", "error": repr(f.exception())}, 500)
    return {"state": "ready", **f.result()}


@router.get("/preview/{design_id}/envelope.stl")
def preview_envelope(design_id: str):
    from .cad.exporters import export_stl
    from .cad.hull import envelope
    from .physics.hullform import HullParams
    if design_id not in _DESIGNS:
        raise HTTPException(404)
    path = EXPORTS / design_id / "envelope.stl"
    if not path.exists():
        path.parent.mkdir(exist_ok=True)
        try:
            with _OCC:
                export_stl(envelope(HullParams(**_DESIGNS[design_id]["params"])), path, 0.3, 0.3)
        except Exception as e:   # surface the real cause in the viewer, not a silent blank
            raise HTTPException(500, f"CAD preview failed: {e!r}")
    return FileResponse(path, media_type="model/stl")


@router.get("/preview/{design_id}/{part}.stl")
def preview_part(design_id: str, part: str):
    if part not in ("hull", "hatch_lid", "rudder") or design_id not in _DESIGNS:
        raise HTTPException(404)
    _schedule(design_id, urgent=True).result()
    return FileResponse(EXPORTS / design_id / f"{part}.stl", media_type="model/stl")


@router.get("/export/{design_id}/{fmt}")
def export(design_id: str, fmt: str):
    names = {"step": "mariforge.step", "stl": "mariforge_stl.zip"}
    if fmt not in names or design_id not in _DESIGNS:
        raise HTTPException(404)
    with _LOCK:
        if design_id not in _EXPORTED:
            f = _EXPORTED[design_id] = Future()
            threading.Thread(target=lambda: _run_export(design_id, f), daemon=True).start()
        f = _EXPORTED[design_id]
    f.result()
    label = _DESIGNS[design_id]["label"].lower()
    ext = "step" if fmt == "step" else "zip"
    return FileResponse(EXPORTS / design_id / names[fmt], filename=f"mariforge_{label}.{ext}")
