@echo off
cd /d %~dp0
if not exist .venv ( py -3.12 -m venv .venv || python -m venv .venv )
.venv\Scripts\python.exe -m pip install -q -r requirements.txt
start "" http://127.0.0.1:8000
.venv\Scripts\python.exe -m uvicorn backend.main:app --host 127.0.0.1 --port 8000
