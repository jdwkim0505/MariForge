import os
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
os.environ.setdefault("MARIFORGE_WARM", "0")   # no background CAD builds under test

DEFAULT_REQ = {"length_mm": 800, "beam_mm": 250, "height_mm": 260,
               "cruise_speed_mps": 0.9, "endurance_min": 30, "bed_mm": 250}

PAYLOAD_REQ = {**DEFAULT_REQ, "payloads": [{"key": k} for k in
               ("autopilot", "telemetry", "echo_sounder", "medical_kit", "fpv_camera",
                "companion_computer")]}
