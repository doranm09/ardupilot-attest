#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
eval "$(opam env --switch=rocq-9.0.1)"

# Capture the ASP request payload for inspection.
export ASP_CAPTURE_FILE="${ROOT_DIR}/outputs/ardupilot_fw_hash_req.json"

cvm \
  --manifest_file "${ROOT_DIR}/manifests/manifest.fw_hash_demo.json" \
  --asp_bin "${ROOT_DIR}/asps" \
  --req_file "${ROOT_DIR}/requests/request.fw_hash_demo.json" \
  --log_level Debug
