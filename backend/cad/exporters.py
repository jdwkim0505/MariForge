"""STEP assembly and STL exports."""
from __future__ import annotations

import io
import json
import zipfile
from pathlib import Path

import cadquery as cq

from .hull import envelope
from ..physics.hullform import HullParams

COLOURS = {"hull": (0.85, 0.87, 0.90), "hatch_lid": (0.20, 0.45, 0.75), "rudder": (0.80, 0.30, 0.20)}
PARTS = ("hull", "hatch_lid", "rudder")


def export_step(parts: dict, path: Path) -> Path:
    asm = cq.Assembly(name="mariforge")
    for k in PARTS:
        asm.add(parts[k], name=k, color=cq.Color(*COLOURS[k]))
    asm.save(str(path), exportType="STEP")
    return path


def export_stl(shape: cq.Shape, path: Path, tol: float = 0.05, ang: float = 0.1) -> Path:
    cq.exporters.export(cq.Workplane().add(shape), str(path), tolerance=tol, angularTolerance=ang)
    return path


def export_stl_zip(design: dict, parts: dict, path: Path, verification: dict | None = None,
                   bed_mm: float = 250.0) -> Path:
    tmp = path.parent
    segs = split_for_bed(design, parts["hull"], bed_mm)
    files = segs + [(k, parts[k]) for k in PARTS if k != "hull"]
    with zipfile.ZipFile(path, "w", zipfile.ZIP_DEFLATED) as z:
        for name, shape in files:
            f = export_stl(shape, tmp / f"_{design['id']}_{name}.stl")
            z.write(f, f"{name}.stl")
            f.unlink()
        env = export_stl(envelope(HullParams(**design["params"])), tmp / f"_{design['id']}_env.stl")
        z.write(env, "hydrostatic_envelope.stl")
        env.unlink()
        manifest = {
            "design_id": design["id"], "label": design["label"],
            "print": {"material": "PLA", "walls": "4 perimeters @ 0.4 mm (1.6 mm)",
                      "hull_orientation": "each hull segment standing on a cut face, so the skin prints as vertical walls with no supports inside"},
            "ballast": design["mass"]["ballast"],
            "parts": [n for n, _ in files],
            "bed_mm": bed_mm,
            "verification": verification,
        }
        z.writestr("manifest.json", json.dumps(manifest, indent=2, default=float))
    return path


def split_for_bed(design: dict, hull: cq.Shape, bed_mm: float = 250.0) -> list[tuple[str, cq.Shape]]:
    """Cut the hull into lengths that fit a print bed, with the cut planes moved
    off the hardware so no fitting is split. Segments butt-join; glue with
    epoxy and run a fillet of thickened epoxy round the seam inside."""
    L = design["params"]["L"]
    n = max(1, int(-(-L // (bed_mm - 5))))
    if n == 1:
        return [("hull", hull)]
    lay = design["layout"]
    keep_out = [(lay["hatch"]["x0"] - 8, lay["hatch"]["x0"] + lay["hatch"]["magnet_pad"] + 2),
                (lay["hatch"]["x1"] - lay["hatch"]["magnet_pad"] - 2, lay["hatch"]["x1"] + 8),
                (lay["servo"]["x"] - lay["servo"]["tray_x"] / 2 - 3, lay["servo"]["x"] + lay["servo"]["tray_x"] / 2 + 3),
                (lay["motor"]["x_face"] - 10, lay["motor"]["x_face"] + 3),
                (lay["rudder"]["x"] - 8, lay["rudder"]["x"] + 8)]
    cuts = []
    for k in range(1, n):
        x = L * k / n
        for a, b in keep_out:
            if a < x < b:
                x = a if x - a < b - x else b
        cuts.append(x)
    edges = [-1.0] + cuts + [L + 1.0]
    big = 10 * max(design["params"]["B"], design["params"]["T"] + design["params"]["F"] + 100)
    segs = []
    for i in range(len(edges) - 1):
        box = cq.Solid.makeBox(edges[i + 1] - edges[i], big, big,
                               cq.Vector(edges[i], -big / 2, -big / 2))
        segs.append((f"hull_seg{i + 1}", hull.intersect(box).clean()))
    return segs
