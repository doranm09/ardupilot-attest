#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
RESP_FILE="${1:-${ROOT_DIR}/outputs/param_hash_demo.response.json}"
KEY="${2:-ardupilot_param_sha256}"
BAD_EXPECTED="${3:-0000000000000000000000000000000000000000000000000000000000000000}"

if [[ ! -f "${RESP_FILE}" ]]; then
  echo "Response file not found: ${RESP_FILE}" >&2
  echo "Run ./scripts/run_param_hash_demo.sh first." >&2
  exit 2
fi

set +e
python3 "${ROOT_DIR}/scripts/verify_raw_ev_measurement.py" \
  --response "${RESP_FILE}" \
  --key "${KEY}" \
  --expected "${BAD_EXPECTED}"
RC=$?
set -e

if [[ ${RC} -eq 2 ]]; then
  echo "Invalid measurement demonstration succeeded (verifier returned INVALID)."
  exit 0
fi

echo "Unexpected verifier exit code: ${RC}" >&2
exit "${RC}"
