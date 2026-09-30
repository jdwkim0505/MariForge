#!/usr/bin/env bash
# Create the venv on first run, install deps, start the server, open the browser.
set -e
cd "$(dirname "$0")"
[ -d .venv ] || python3 -m venv .venv
.venv/bin/pip install -q -r requirements.txt
( sleep 3; (xdg-open http://127.0.0.1:8000 || open http://127.0.0.1:8000) >/dev/null 2>&1 ) &
exec .venv/bin/python -m uvicorn backend.main:app --host 127.0.0.1 --port 8000
