#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

if ! command -v python3 >/dev/null 2>&1; then
  echo "python3 not found in PATH." >&2
  exit 2
fi

if ! python3 -c 'import pymavlink' >/dev/null 2>&1; then
  cat >&2 <<'EOF'
pymavlink is not available for python3.
Activate the ArduPilot venv (or another venv that has pymavlink) first.
EOF
  exit 2
fi

python3 "${ROOT_DIR}/scripts/sitl_rtl_on_attestation_fail.py" "$@"
