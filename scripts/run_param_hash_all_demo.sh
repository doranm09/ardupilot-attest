#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
LOG_FILE="${ROOT_DIR}/outputs/param_hash_all_demo.log"
RESP_FILE="${ROOT_DIR}/outputs/param_hash_all_demo.response.json"
SWITCH_NAME="${SWITCH_NAME:-rocq-9.0.1}"

mkdir -p "${ROOT_DIR}/outputs"
eval "$(opam env --switch="${SWITCH_NAME}")"

if ! command -v cvm >/dev/null 2>&1; then
  echo "cvm not found in PATH. Run ./scripts/use-cvm-env.sh first." >&2
  exit 2
fi

if ! command -v python3 >/dev/null 2>&1; then
  echo "python3 not found in PATH." >&2
  exit 2
fi

if ! python3 -c 'import pymavlink' >/dev/null 2>&1; then
  cat >&2 <<'EOF2'
pymavlink is not available for python3.
Install it in a virtual environment and activate that shell:
  python3 -m venv .venv
  source .venv/bin/activate
  pip install pymavlink
EOF2
  exit 2
fi

echo "Running CVM all-parameter hash demo (log: ${LOG_FILE})"
cvm \
  --manifest_file "${ROOT_DIR}/manifests/manifest.param_hash_demo.json" \
  --asp_bin "${ROOT_DIR}/asps" \
  --req_file "${ROOT_DIR}/requests/request.param_hash_all_demo.json" \
  --log_level Debug | tee "${LOG_FILE}"

if rg -q "errStr_raw_EvidenceT_too_long" "${LOG_FILE}"; then
  cat >&2 <<'EOF3'
CVM rejected ASP output length (errStr_raw_EvidenceT_too_long).
Fix: set request ATTESTATION_SESSION.Session_Context.ASP_Types.<ASP_ID>.EvOutSig.OutN
to exactly the number of RawEv entries returned by the ASP.
EOF3
  exit 1
fi

if ! python3 "${ROOT_DIR}/scripts/extract_last_json.py" "${LOG_FILE}" "${RESP_FILE}"; then
  echo "Failed to extract CVM JSON response from ${LOG_FILE}. Last 40 log lines:" >&2
  tail -n 40 "${LOG_FILE}" >&2 || true
  exit 1
fi
echo "Wrote response JSON: ${RESP_FILE}"
