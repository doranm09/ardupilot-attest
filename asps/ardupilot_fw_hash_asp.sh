#!/usr/bin/env bash
set -euo pipefail

# Demo ASP: read request JSON on stdin and return a mock raw evidence measurement.
# Replace this with real ArduPilot measurement logic.
cat > "${ASP_CAPTURE_FILE:-/tmp/ardupilot_fw_hash_req.json}" || true
printf '%s\n' '{"TYPE":"RESPONSE","ACTION":"ASP_RUN","SUCCESS":true,"PAYLOAD":{"RawEv":["fw_sha256=demo"]}}'
