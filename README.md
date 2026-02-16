# ArduPilot Attestation Starter

This directory provides a CVM starter scaffold for ArduPilot-focused attestation work.

## Contents

- `asps/ardupilot_fw_hash_asp.sh`: demo ASP executable (stdin -> stdout JSON)
- `asps/ardupilot_param_hash_asp.py`: real MAVLink parameter-measurement ASP
- `asps/ardupilot_mission_hash_asp.py`: MAVLink mission-waypoint measurement ASP
- `manifests/manifest.null.json`: no-ASP manifest for parser/smoke test
- `manifests/manifest.fw_hash_demo.json`: manifest mapping `ardupilot_fw_hash` to demo ASP
- `manifests/manifest.param_hash_demo.json`: manifest mapping `ardupilot_param_hash` to MAVLink ASP
- `manifests/manifest.mission_hash_demo.json`: manifest mapping `ardupilot_mission_hash` to MAVLink ASP
- `requests/request.null.json`: RUN request with `asp NULL`
- `requests/request.fw_hash_demo.json`: RUN request with `asp ASPC(...)` calling demo ASP
- `requests/request.param_hash_demo.json`: RUN request calling the parameter-hash ASP
- `requests/request.param_hash_all_demo.json`: RUN request calling parameter-hash ASP over all available params
- `requests/request.mission_hash_demo.json`: RUN request calling the mission-hash ASP
- `scripts/use-cvm-env.sh`: activates `rocq-9.0.1` OPAM env and validates tool paths
- `scripts/run_null_smoke.sh`: executes null-term smoke test
- `scripts/run_fw_hash_demo.sh`: executes demo ASP flow and captures ASP stdin payload
- `scripts/run_param_hash_demo.sh`: runs real parameter measurement and extracts response JSON
- `scripts/run_param_hash_all_demo.sh`: runs full-parameter measurement and extracts response JSON
- `scripts/run_mission_hash_demo.sh`: runs real mission-waypoint measurement and extracts response JSON
- `scripts/capture_param_hash_baseline.sh`: captures and stores parameter baseline hash
- `scripts/capture_param_hash_all_baseline.sh`: captures and stores full-parameter baseline hash
- `scripts/capture_mission_hash_baseline.sh`: captures and stores mission baseline hash
- `scripts/extract_last_json.py`: extracts final response object from CVM logs
- `scripts/verify_raw_ev_measurement.py`: validates `RawEv` entries against expected/baseline values
- `scripts/demo_invalid_measurement.sh`: forces an expected mismatch and demonstrates invalid appraisal
- `scripts/sitl_rtl_on_attestation_fail.py`: uploads mission, runs attestation, triggers RTL on invalid appraisal
- `scripts/run_sitl_rtl_fail_demo.sh`: shell wrapper for the SITL RTL-fail demo
- `scripts/run_sitl_map_attest_demo.sh`: launches SITL (map/console) and runs mission+attestation gate using `mission7.waypoints`
- `outputs/`: runtime artifacts

## Prerequisites

- OPAM switch `rocq-9.0.1`
- `cvm` installed in that switch (`opam install .` from `/home/michaeldoran/git/copland/cvm`)
- Python 3 with `pymavlink` for real ArduPilot parameter measurements
- A MAVLink endpoint (default in request: `udp:127.0.0.1:14550`)

## Install `pymavlink` (recommended: virtual environment)

```bash
cd /home/michaeldoran/git/copland/ardupilot-attest
python3 -m venv .venv
source .venv/bin/activate
pip install --upgrade pip
pip install pymavlink
```

If your system Python is externally managed (PEP 668), use the venv path above.

## Quick Start (existing demo flows)

```bash
cd /home/michaeldoran/git/copland/ardupilot-attest
./scripts/use-cvm-env.sh
./scripts/run_null_smoke.sh > outputs/null_smoke.log 2>&1
./scripts/run_fw_hash_demo.sh > outputs/fw_hash_demo.log 2>&1
```

## Real Parameter Measurement Flow

1. Start/attach to ArduPilot MAVLink at `udp:127.0.0.1:14550` (or update `requests/request.param_hash_demo.json`).
2. Ensure `python3` used by the ASP can import `pymavlink` (activate `.venv` if needed).
3. Run:

```bash
cd /home/michaeldoran/git/copland/ardupilot-attest
source .venv/bin/activate
./scripts/run_param_hash_demo.sh
```

Outputs:
- `outputs/param_hash_demo.log`
- `outputs/param_hash_demo.response.json`

Note:
- Current selected-profile request tracks operational/failsafe parameters:
  - `WPNAV_SPEED`, `WPNAV_ACCEL`, `RTL_ALT`
  - `FS_THR_ENABLE`, `FS_GCS_ENABLE`, `BATT_FS_LOW_ACT`, `FENCE_ENABLE`
- Some ArduPilot versions/vehicle types do not expose older names like `SYSID_THISMAV` / `SYSID_MYGCS`.
- To inspect available names from MAVProxy output: `rg '^(WPNAV_|RTL_|FS_|BATT_|FENCE_)' ~/git/ardupilot/mav.parm`
- `EvOutSig.OutN` must exactly match ASP `RawEv` entry count. This demo uses `OutN=3`.

## Full Parameter Measurement Flow (all available params)

Run full-parameter profile:

```bash
cd /home/michaeldoran/git/copland/ardupilot-attest
source .venv/bin/activate
./scripts/run_param_hash_all_demo.sh
```

Outputs:
- `outputs/param_hash_all_demo.log`
- `outputs/param_hash_all_demo.response.json`

Default stabilization filters in `requests/request.param_hash_all_demo.json`:
- `exclude_prefixes: ["STAT_"]` to ignore runtime counters that naturally drift during flight.
- `exclude_names: ["MIS_TOTAL"]` to avoid mission-count coupling in the parameter hash (mission integrity is already attested separately).
- In the current SITL run this excludes: `MIS_TOTAL`, `STAT_BOOTCNT`, `STAT_DISTFLWN`, `STAT_FLTCNT`, `STAT_FLTTIME`, `STAT_RESET`, `STAT_RUNTIME`.

If you need a different policy, edit those two fields and then recapture `outputs/param_hash_all_baseline.sha256`.

## Mission Waypoint Measurement Flow

1. Ensure SITL is running and a mission is loaded.
2. Run:

```bash
cd /home/michaeldoran/git/copland/ardupilot-attest
source .venv/bin/activate
./scripts/run_mission_hash_demo.sh
```

Outputs:
- `outputs/mission_hash_demo.log`
- `outputs/mission_hash_demo.response.json`
- `outputs/mission_hash_demo.canonical.txt` (canonical mission lines for diff/debug)

Note:
- Mission hash ignores `MISSION_ITEM.current` (execution state).
- Mission hash normalizes takeoff position (`MAV_CMD_NAV_TAKEOFF`) to keep baselines stable across switching to `AUTO` / takeoff when ArduPilot rewrites the takeoff item.
- You can diff the plan across phases (ground vs takeoff) using `outputs/mission_hash_demo.canonical.txt`.

## Capture Baselines (for real pass/fail appraisal)

Capture parameter baseline:

```bash
./scripts/capture_param_hash_baseline.sh
```

Capture full-parameter baseline:

```bash
./scripts/capture_param_hash_all_baseline.sh
```

Capture mission baseline (after loading your intended mission):

```bash
./scripts/capture_mission_hash_baseline.sh
```

Baseline files used automatically by the SITL demo when present:
- `outputs/param_hash_baseline.sha256`
- `outputs/param_hash_all_baseline.sha256`
- `outputs/mission_hash_baseline.sha256`

## Measurement Validation and Invalid Demonstration

Check a measured value against expected:

```bash
python3 scripts/verify_raw_ev_measurement.py \
  --response outputs/param_hash_demo.response.json \
  --key ardupilot_param_sha256 \
  --expected <known-good-sha256>
```

Deliberately show invalid measurement handling:

```bash
./scripts/demo_invalid_measurement.sh
```

The demo passes when verifier exits with status `2` and reports `INVALID`.

## SITL Mission + RTL on Attestation Failure

Default sequence in this demo:
- Upload mission
- Arm and take off
- Enter `AUTO` and wait for observable mission progress
- Run appraisal on both parameter hash and mission hash
- If either appraisal is `INVALID`, command `RTL`

### Option A: attach to already-running SITL (recommended for interactive visual map)

Terminal 1:
```bash
cd ~/git/ardupilot/ArduCopter
./start-sim.sh single
```

Terminal 2:
```bash
cd /home/michaeldoran/git/copland/ardupilot-attest
source ~/git/ardupilot/venv-ardupilot/bin/activate
./scripts/run_sitl_map_attest_demo.sh --no-launch-sitl
```

### Option B: one command (launch SITL + map/console + attestation)

```bash
cd /home/michaeldoran/git/copland/ardupilot-attest
source ~/git/ardupilot/venv-ardupilot/bin/activate
./scripts/run_sitl_map_attest_demo.sh
```

Launcher resolution order:
- `~/git/ardupilot/ArduCopter/run_sitl.py` (if present)
- `~/git/ardupilot/ArduCopter/start-sim.sh` (preferred when `--master udp:127.0.0.1:14550`)
- `~/git/ardupilot/ArduCopter/run_sitl.sh`
- `~/git/ardupilot/Tools/autotest/sim_vehicle.py`

Default mission file:
- `~/git/CRABWAVE/MP2/mission7.waypoints`

Useful options:

```bash
# Keep mission running on valid appraisal
./scripts/run_sitl_map_attest_demo.sh --pass-attestation

# Use full-parameter profile instead of the selected operational/failsafe set
./scripts/run_sitl_map_attest_demo.sh --param-profile all

# Use a different endpoint
./scripts/run_sitl_map_attest_demo.sh --master udp:127.0.0.1:14551

# Leave SITL running after demo exits
./scripts/run_sitl_map_attest_demo.sh --keep-sitl

# Give yourself more time to change params/mission in MAVProxy before appraisal
./scripts/run_sitl_map_attest_demo.sh --pre-attestation-timeout 90

# Periodically re-attest every 15s (run until invalid, then RTL)
./scripts/run_sitl_map_attest_demo.sh --periodic-attestation --attestation-interval 15

# Run exactly 3 periodic cycles
./scripts/run_sitl_map_attest_demo.sh --periodic-attestation --attestation-interval 10 --attestation-cycles 3
```

Additional automation controls:
- `--skip-mission-upload`: when you want to prepare/upload waypoints manually (MAVProxy) before running the demo.
- `--skip-auto-flight`: skip the guided takeoff/mission start sequence and let your MAVProxy session arm/takeoff/time the mission; the script will still connect and run attestation cycles afterward.

Notes:
- Script tries full `AUTO` mission first; if mode change is rejected by SITL state, it falls back to a `GUIDED` leg, then still runs appraisal.
- Default mode is one-shot appraisal; periodic mode is enabled with `--periodic-attestation`.
- If baseline files are missing, default mode is a forced fail (`expected=00..00`) to demonstrate RTL.
- On some ArduPilot builds, mission upload can return a non-accepted ACK while still loading the plan; script cross-checks mission count and continues when count matches.

## MAVProxy Tamper Demo (real failure, not forced)

1. Capture both baselines first (`capture_param_hash_baseline.sh` and `capture_mission_hash_baseline.sh`).
2. Start mission with `run_sitl_map_attest_demo.sh --no-launch-sitl`.
3. In MAVProxy, change a measured value, for example:

```text
param set WPNAV_SPEED 1200
param fetch WPNAV_SPEED
```

4. Re-run appraisal. Parameter hash no longer matches baseline, so result becomes `INVALID` and the script commands `RTL`.

Mission tamper option:
- Load a different waypoint file in MAVProxy (for example with `wp load ...`) and re-run appraisal; mission hash mismatch also triggers `INVALID` -> `RTL`.

For full parameter coverage:
- Use `--param-profile all` and capture `capture_param_hash_all_baseline.sh` before the mission run.

## Manifest `POLICY` Guidance

- `POLICY` is an array of deny-rules, each rule is a pair: `[plc, asp_id]`.
- Example:

```json
"POLICY": [
  ["ground", "ardupilot_param_hash"]
]
```

- Keep `POLICY: []` for initial bring-up, then add rules once remote disclosure behavior is mapped.
