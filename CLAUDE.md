# CLAUDE.md

Guidance for Claude Code working in this repository.

## What MariForge is

The maritime sibling of AeroForge (github.com/stekimboy/aeroforge). A local web
app that designs **RC displacement hulls**: give it a size box, a cruise speed
and an endurance; it solves the coupled hydrostatics / weight / resistance /
propulsion problem, returns five genuinely different hulls, renders them in 3D
and exports STEP / STL with the hardware already in the part.

## Doctrine (carried over from AeroForge, non-negotiable)

- **Every number is a real calculation.** No placeholder physics.
- **Prove it in the exported geometry, never in builder flags.** The envelope
  STL must displace the design volume at z = 0 and float there under the
  design mass; every bore, pocket and groove is checked by point
  classification of the finished solid (`backend/cad/verify.py`).
- **A design that cannot carry a feature says so by name.** Layout checks
  (`physics/layout.py`) that fail make the builder skip that feature and list
  it under `refused`; the design is marked infeasible with the reason. Never
  ship a design silently missing a part.
- **One source of truth for shape and placement.** `physics/hullform.py`
  defines the surface; `physics/layout.py` places every fitting. Physics and
  CAD both read them. Do not duplicate a dimension in the CAD code.

## Frame and units

x from the transom (0) to the stem (L), y to port, z up with the **design
waterline at z = 0**. CAD and layout are in mm; resistance is SI.

## Commands

```
./run.sh                                             # venv + deps + server + browser (run.bat on Windows)
.venv/bin/python -m uvicorn backend.main:app --port 8000
.venv/bin/python -m pytest tests/ -q                 # everything (~10 min; CAD dominates)
.venv/bin/python -m pytest tests/ -q --ignore=tests/test_cad.py   # physics + API (~3 min)
.venv/bin/python -m pytest tests/test_cad.py -q -k cruiser        # one full CAD build
```

## Where things live

| module | what |
|---|---|
| `physics/hullform.py` | parametric section definition, cavity offset |
| `physics/hydrostatics.py` | section integration: volume, LCB/KB, waterplane, fixed-trim GZ, downflooding |
| `physics/resistance.py` | ITTC-57 + Holtrop-Mennen (Fn <= 0.45) |
| `physics/components.py` | motors, batteries, fixed parts, PLA, wall |
| `physics/layout.py` | hatch, rudder, prop/shaft, motor mount, servo thwart, battery; named checks |
| `physics/weights.py` | mass budget, CG, ballast solve for level trim |
| `physics/design.py` | the coupled solve for one hull + all checks |
| `physics/variants.py` | the five characters and the gallery traits |
| `cad/hull.py` | CadQuery lofts and features (one fuse, one cut) |
| `cad/verify.py` | mesh hydrostatics + cut existence checks |
| `cad/exporters.py` | STEP assembly, STL zip, split for the print bed |
| `api.py` | FastAPI; one CAD worker thread, the viewed design jumps the queue |

## Open items

See README "Known gaps".
