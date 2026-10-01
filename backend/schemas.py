from pydantic import BaseModel, Field, model_validator

from .physics.components import PAYLOADS


class PayloadRow(BaseModel):
    key: str = "custom"
    qty: int = Field(1, ge=1, le=6)
    name: str | None = None
    mass_g: float | None = Field(None, gt=0, le=5000)
    dims_mm: list[float] | None = None
    power_W: float = Field(0.0, ge=0, le=60)
    mount: str = "bay"
    transducer_dia_mm: float | None = None

    @model_validator(mode="after")
    def _known_or_complete(self):
        if self.key in PAYLOADS:
            return self
        if self.mass_g is None or not self.dims_mm or len(self.dims_mm) != 3:
            raise ValueError(f"payload '{self.key}' is not in the catalogue: give mass_g and dims_mm [l, w, h]")
        if any(d <= 0 or d > 1000 for d in self.dims_mm):
            raise ValueError("payload dims_mm must be between 0 and 1000 mm")
        if self.mount not in ("bay", "deck", "through_hull"):
            raise ValueError("payload mount must be bay, deck or through_hull")
        return self


class GenerateRequest(BaseModel):
    hull_type: str = Field("displacement_monohull")
    mission: str = Field("scale_cruiser")
    length_mm: float = Field(800, ge=350, le=1600)
    beam_mm: float = Field(250, ge=80, le=600)
    height_mm: float = Field(260, ge=80, le=600)
    cruise_speed_mps: float = Field(0.9, gt=0.1, le=4.0)
    endurance_min: float = Field(30, gt=0, le=600)
    bed_mm: float = Field(250, ge=150, le=600)
    wave_height_mm: float = Field(0, ge=0, le=500)
    payloads: list[PayloadRow] = Field(default_factory=list)
