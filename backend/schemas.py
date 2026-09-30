from pydantic import BaseModel, Field


class GenerateRequest(BaseModel):
    hull_type: str = Field("displacement_monohull")
    mission: str = Field("scale_cruiser")
    length_mm: float = Field(800, ge=350, le=1600)
    beam_mm: float = Field(250, ge=80, le=600)
    height_mm: float = Field(260, ge=80, le=600)
    cruise_speed_mps: float = Field(0.9, gt=0.1, le=4.0)
    endurance_min: float = Field(30, gt=0, le=600)
    bed_mm: float = Field(250, ge=150, le=600)
