#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

MASTER="udp:127.0.0.1:14550"
MISSION_FILE="$HOME/git/CRABWAVE/MP2/mission7.waypoints"
START_WAIT_SEC="18"
HEARTBEAT_TIMEOUT_SEC="90"
NO_LAUNCH_SITL=0
KEEP_SITL=0
TMUX_SESSION_NAME=""
ATT_ARGS=()

usage() {
  cat <<'USAGE'
Run ArduPilot SITL (with map/console) and execute attestation mission gate.

Defaults:
  master            udp:127.0.0.1:14550
  mission file      ~/git/CRABWAVE/MP2/mission7.waypoints
  launcher mode     start SITL via custom launcher if available

Usage:
  ./scripts/run_sitl_map_attest_demo.sh [options] [-- <extra attestation args>]

Options:
  --master <endpoint>         MAVLink endpoint for SITL + attestation script.
  --mission-file <path>       QGC WPL 110 mission file.
  --start-wait-sec <n>        Seconds to wait before attestation starts.
  --heartbeat-timeout <n>     Seconds for first heartbeat in attestation script.
  --no-launch-sitl            Do not start SITL; attach to an already running endpoint.
  --keep-sitl                 Leave started SITL running after demo exits.
  --tmux-session <name>       Session name if tmux launch is used.
  -h, --help                  Show this help.

Examples:
  ./scripts/run_sitl_map_attest_demo.sh
  ./scripts/run_sitl_map_attest_demo.sh --pass-attestation
  ./scripts/run_sitl_map_attest_demo.sh --no-launch-sitl
USAGE
}

shell_join() {
  local out=""
  local q
  for arg in "$@"; do
    printf -v q '%q' "$arg"
    out+="$q "
  done
  printf '%s' "$out"
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --master)
      MASTER="${2:-}"
      shift 2
      ;;
    --mission-file)
      MISSION_FILE="${2:-}"
      shift 2
      ;;
    --start-wait-sec)
      START_WAIT_SEC="${2:-}"
      shift 2
      ;;
    --heartbeat-timeout)
      HEARTBEAT_TIMEOUT_SEC="${2:-}"
      shift 2
      ;;
    --no-launch-sitl)
      NO_LAUNCH_SITL=1
      shift
      ;;
    --keep-sitl)
      KEEP_SITL=1
      shift
      ;;
    --tmux-session)
      TMUX_SESSION_NAME="${2:-}"
      shift 2
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    --)
      shift
      ATT_ARGS+=("$@")
      break
      ;;
    *)
      ATT_ARGS+=("$1")
      shift
      ;;
  esac
done

if ! command -v python3 >/dev/null 2>&1; then
  echo "python3 not found in PATH." >&2
  exit 2
fi

if ! python3 -c 'import pymavlink' >/dev/null 2>&1; then
  cat >&2 <<'EOF2'
pymavlink is not available for python3.
Activate the ArduPilot venv (or another venv that has pymavlink) first.
EOF2
  exit 2
fi

mkdir -p "$ROOT_DIR/outputs"
SITL_LOG="$ROOT_DIR/outputs/sitl_map_attest_demo.sitl.log"
LAUNCH_MODE="none"
SITL_PID=""

cleanup() {
  if [[ "$KEEP_SITL" -eq 1 ]]; then
    if [[ "$LAUNCH_MODE" == "tmux" ]]; then
      echo "[INFO] Keeping SITL running in tmux session '$TMUX_SESSION_NAME'."
    elif [[ "$LAUNCH_MODE" == "setsid" && -n "$SITL_PID" ]]; then
      echo "[INFO] Keeping SITL running (process group -$SITL_PID)."
    fi
    return
  fi

  if [[ "$LAUNCH_MODE" == "tmux" && -n "$TMUX_SESSION_NAME" ]]; then
    tmux kill-session -t "$TMUX_SESSION_NAME" 2>/dev/null || true
    echo "[INFO] Stopped SITL tmux session '$TMUX_SESSION_NAME'."
  elif [[ "$LAUNCH_MODE" == "setsid" && -n "$SITL_PID" ]]; then
    kill -TERM "-$SITL_PID" 2>/dev/null || true
    sleep 1
    kill -KILL "-$SITL_PID" 2>/dev/null || true
    echo "[INFO] Stopped SITL process group -$SITL_PID"
  fi
}
trap cleanup EXIT INT TERM

if [[ "$NO_LAUNCH_SITL" -eq 0 ]]; then
  CUSTOM_RUN_PY="$HOME/git/ardupilot/ArduCopter/run_sitl.py"
  CUSTOM_START_SIM_SH="$HOME/git/ardupilot/ArduCopter/start-sim.sh"
  CUSTOM_RUN_SH="$HOME/git/ardupilot/ArduCopter/run_sitl.sh"
  SIM_VEHICLE_PY="$HOME/git/ardupilot/Tools/autotest/sim_vehicle.py"

  SITL_CMD=()
  SITL_WORKDIR="$HOME/git/ardupilot"
  SITL_FLAVOR=""

  if [[ -f "$CUSTOM_RUN_PY" ]]; then
    SITL_CMD=(python3 "$CUSTOM_RUN_PY")
    SITL_WORKDIR="$(dirname "$CUSTOM_RUN_PY")"
    SITL_FLAVOR="run_sitl.py"
  elif [[ -x "$CUSTOM_START_SIM_SH" && "$MASTER" == "udp:127.0.0.1:14550" ]]; then
    SITL_CMD=(bash "$CUSTOM_START_SIM_SH" single)
    SITL_WORKDIR="$(dirname "$CUSTOM_START_SIM_SH")"
    SITL_FLAVOR="start-sim.sh"
  elif [[ -x "$CUSTOM_START_SIM_SH" && "$MASTER" != "udp:127.0.0.1:14550" ]]; then
    echo "[WARN] start-sim.sh selected but --master is non-default ($MASTER); falling back to run_sitl.sh for endpoint control."
    if [[ -x "$CUSTOM_RUN_SH" ]]; then
      SITL_CMD=(bash "$CUSTOM_RUN_SH" -- --out "$MASTER")
      SITL_WORKDIR="$(dirname "$CUSTOM_RUN_SH")"
      SITL_FLAVOR="run_sitl.sh"
    elif [[ -f "$SIM_VEHICLE_PY" ]]; then
      SITL_CMD=(python3 "$SIM_VEHICLE_PY" -v ArduCopter --map --console --out "$MASTER")
      SITL_WORKDIR="$HOME/git/ardupilot"
      SITL_FLAVOR="sim_vehicle.py"
    fi
  elif [[ -x "$CUSTOM_RUN_SH" ]]; then
    SITL_CMD=(bash "$CUSTOM_RUN_SH" -- --out "$MASTER")
    SITL_WORKDIR="$(dirname "$CUSTOM_RUN_SH")"
    SITL_FLAVOR="run_sitl.sh"
  elif [[ -f "$SIM_VEHICLE_PY" ]]; then
    SITL_CMD=(python3 "$SIM_VEHICLE_PY" -v ArduCopter --map --console --out "$MASTER")
    SITL_WORKDIR="$HOME/git/ardupilot"
    SITL_FLAVOR="sim_vehicle.py"
  else
    echo "Could not find any SITL launcher at expected paths:" >&2
    echo "  $CUSTOM_RUN_PY" >&2
    echo "  $CUSTOM_START_SIM_SH" >&2
    echo "  $CUSTOM_RUN_SH" >&2
    echo "  $SIM_VEHICLE_PY" >&2
    exit 2
  fi

  echo "[INFO] Starting SITL via $SITL_FLAVOR"
  echo "[INFO] Command: ${SITL_CMD[*]}"
  echo "[INFO] SITL log: $SITL_LOG"

  if command -v tmux >/dev/null 2>&1; then
    if [[ -z "$TMUX_SESSION_NAME" ]]; then
      TMUX_SESSION_NAME="sitl_attest_$(date +%H%M%S)"
    fi
    cmdline="$(shell_join "${SITL_CMD[@]}")"
    workdir_q="$(printf '%q' "$SITL_WORKDIR")"
    log_q="$(printf '%q' "$SITL_LOG")"
    tmux new-session -d -s "$TMUX_SESSION_NAME" \
      "cd $workdir_q && export SITL_RITW_TERMINAL='bash -lc' && exec $cmdline > $log_q 2>&1"
    LAUNCH_MODE="tmux"
    echo "[INFO] SITL running in tmux session '$TMUX_SESSION_NAME'."
  else
    pushd "$SITL_WORKDIR" >/dev/null
    setsid env SITL_RITW_TERMINAL="bash -lc" "${SITL_CMD[@]}" >"$SITL_LOG" 2>&1 &
    SITL_PID=$!
    popd >/dev/null
    LAUNCH_MODE="setsid"
    echo "[INFO] SITL started with PID $SITL_PID (process group)."
  fi

  sleep "$START_WAIT_SEC"
else
  echo "[INFO] --no-launch-sitl set; attaching to existing endpoint $MASTER"
fi

python3 "$ROOT_DIR/scripts/sitl_rtl_on_attestation_fail.py" \
  --master "$MASTER" \
  --heartbeat-timeout "$HEARTBEAT_TIMEOUT_SEC" \
  --mission-file "$MISSION_FILE" \
  "${ATT_ARGS[@]}"
