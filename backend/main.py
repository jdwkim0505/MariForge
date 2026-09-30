from pathlib import Path

from fastapi import FastAPI
from fastapi.staticfiles import StaticFiles

from .api import router

app = FastAPI(title="MariForge")
app.include_router(router)
app.mount("/", StaticFiles(directory=Path(__file__).resolve().parent.parent / "frontend", html=True),
          name="frontend")
