#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
RESP_JSON="$ROOT_DIR/outputs/mission_hash_demo.response.json"
BASELINE_FILE="$ROOT_DIR/outputs/mission_hash_baseline.sha256"

"$ROOT_DIR/scripts/run_mission_hash_demo.sh"

ROOT_DIR="$ROOT_DIR" python3 - <<'PY'
import json, pathlib
import os
root = pathlib.Path(os.environ['ROOT_DIR'])
resp = root / 'outputs' / 'mission_hash_demo.response.json'
out = root / 'outputs' / 'mission_hash_baseline.sha256'
obj = json.loads(resp.read_text(encoding='utf-8'))
raw = obj.get('PAYLOAD', [{}])[0].get('RawEv', [])
val = None
for item in raw:
    if isinstance(item, str) and item.startswith('ardupilot_mission_sha256='):
        val = item.split('=', 1)[1].strip()
        break
if not val:
    raise SystemExit('Failed to find ardupilot_mission_sha256 in response JSON.')
out.write_text(val + '\n', encoding='utf-8')
print(f'Mission baseline hash: {val}')
print(f'Wrote: {out}')
PY

echo "Response JSON: $RESP_JSON"
echo "Baseline file: $BASELINE_FILE"
