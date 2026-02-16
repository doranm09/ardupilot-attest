#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
eval "$(opam env --switch=rocq-9.0.1)"

cvm \
  --manifest_file "${ROOT_DIR}/manifests/manifest.null.json" \
  --asp_bin "/tmp" \
  --req_file "${ROOT_DIR}/requests/request.null.json" \
  --log_level Debug
