#!/usr/bin/env python3
"""
SITL demo: start mission, run attestation, force RTL when attestation fails.

Default behavior intentionally fails appraisal by comparing against an invalid
expected hash (all zeros), then sends mode RTL.
"""

from __future__ import annotations

import argparse
import json
import math
import os
import pathlib
import subprocess
import sys
import time
from typing import Dict, List

from pymavlink import mavutil


def _run(cmd: List[str], cwd: pathlib.Path) -> subprocess.CompletedProcess[str]:
    return subprocess.run(cmd, cwd=str(cwd), text=True, capture_output=True)


def _wait_heartbeat(master, timeout: float) -> None:
    hb = master.wait_heartbeat(timeout=timeout)
    if hb is None:
        raise RuntimeError("No heartbeat received from vehicle.")


def _norm_pid(v) -> str:
    if isinstance(v, bytes):
        txt = v.decode("utf-8", errors="ignore")
    else:
        txt = str(v)
    return txt.split("\x00", 1)[0].strip().upper()


def _set_param(master, name: str, value: float, timeout: float = 12.0) -> None:
    pid = name.strip().upper()
    master.mav.param_set_send(
        master.target_system,
        master.target_component,
        pid.encode("utf-8"),
        float(value),
        mavutil.mavlink.MAV_PARAM_TYPE_REAL32,
    )
    next_read = 0.0
    start = time.time()
    end = start + timeout
    while time.time() < end:
        now = time.time()
        if now >= next_read:
            master.mav.param_request_read_send(
                master.target_system, master.target_component, pid.encode("utf-8"), -1
            )
            next_read = now + 1.0
        msg = master.recv_match(type="PARAM_VALUE", blocking=True, timeout=0.6)
        if msg is None:
            continue
        if _norm_pid(getattr(msg, "param_id", "")) == pid and abs(
            float(getattr(msg, "param_value", 0.0)) - float(value)
        ) < 0.5:
            return
    raise RuntimeError(f"Timeout setting parameter {pid}.")


def _mode_map(master) -> Dict[str, int]:
    mm = master.mode_mapping()
    if not mm:
        raise RuntimeError("Vehicle did not provide mode mapping.")
    return {k.upper(): int(v) for k, v in mm.items()}


def _set_mode(master, mode_name: str, timeout: float = 12.0) -> None:
    mm = _mode_map(master)
    want = mode_name.upper()
    if want not in mm:
        raise RuntimeError(f"Mode '{mode_name}' not found. Available: {sorted(mm.keys())}")
    mode_id = mm[want]
    # Try SET_MODE first.
    master.mav.set_mode_send(
        master.target_system,
        mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
        mode_id,
    )
    # Also try command-long mode set for stacks that prefer it.
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_DO_SET_MODE,
        0,
        float(mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED),
        float(mode_id),
        0,
        0,
        0,
        0,
        0,
    )
    start = time.time()
    end = start + timeout
    last_statustext = None
    while time.time() < end:
        msg = master.recv_match(
            type=["HEARTBEAT", "STATUSTEXT", "COMMAND_ACK"], blocking=True, timeout=0.8
        )
        if msg is None:
            continue
        mtype = msg.get_type()
        if mtype == "HEARTBEAT" and int(getattr(msg, "custom_mode", -1)) == mode_id:
            return
        if mtype == "STATUSTEXT":
            txt = getattr(msg, "text", "")
            if isinstance(txt, bytes):
                txt = txt.decode("utf-8", errors="ignore")
            last_statustext = str(txt)
    if last_statustext:
        raise RuntimeError(f"Timeout waiting for mode {mode_name}. Last STATUSTEXT: {last_statustext}")
    raise RuntimeError(f"Timeout waiting for mode {mode_name}.")


def _arm(master, timeout: float = 20.0, force_after: float = 8.0) -> None:
    def send_arm(force: bool) -> None:
        # param2=21196 requests force-arm in ArduPilot.
        param2 = 21196.0 if force else 0.0
        master.mav.command_long_send(
            master.target_system,
            master.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0,
            1,
            param2,
            0,
            0,
            0,
            0,
            0,
        )

    start = time.time()
    end = start + timeout
    next_send = 0.0
    last_status = None
    while time.time() < end:
        now = time.time()
        force = (now - start) >= force_after
        if now >= next_send:
            send_arm(force=force)
            next_send = now + 2.0
        if master.motors_armed():
            return
        msg = master.recv_match(
            type=["HEARTBEAT", "STATUSTEXT", "COMMAND_ACK"], blocking=True, timeout=0.8
        )
        if msg is None:
            continue
        if msg.get_type() == "STATUSTEXT":
            txt = getattr(msg, "text", "")
            if isinstance(txt, bytes):
                txt = txt.decode("utf-8", errors="ignore")
            last_status = str(txt)
    if last_status:
        raise RuntimeError(f"Timeout waiting for vehicle to arm. Last STATUSTEXT: {last_status}")
    raise RuntimeError("Timeout waiting for vehicle to arm.")


def _get_position(master, timeout: float = 10.0) -> tuple[float, float]:
    end = time.time() + timeout
    while time.time() < end:
        msg = master.recv_match(type="GLOBAL_POSITION_INT", blocking=True, timeout=1.0)
        if msg is None:
            continue
        lat = float(msg.lat) / 1e7
        lon = float(msg.lon) / 1e7
        if abs(lat) > 0.001 and abs(lon) > 0.001:
            return lat, lon
    raise RuntimeError("Could not read GLOBAL_POSITION_INT for mission upload.")


def _mission_clear_all(master) -> None:
    try:
        master.mav.mission_clear_all_send(
            master.target_system,
            master.target_component,
            mavutil.mavlink.MAV_MISSION_TYPE_MISSION,
        )
    except TypeError:
        master.mav.mission_clear_all_send(master.target_system, master.target_component)


def _mission_count_send(master, count: int) -> None:
    try:
        master.mav.mission_count_send(
            master.target_system,
            master.target_component,
            count,
            mavutil.mavlink.MAV_MISSION_TYPE_MISSION,
        )
    except TypeError:
        master.mav.mission_count_send(master.target_system, master.target_component, count)


def _mission_request_list_send(master) -> None:
    try:
        master.mav.mission_request_list_send(
            master.target_system,
            master.target_component,
            mavutil.mavlink.MAV_MISSION_TYPE_MISSION,
        )
    except TypeError:
        master.mav.mission_request_list_send(master.target_system, master.target_component)


def _read_mission_count(master, timeout: float = 3.0) -> int | None:
    end = time.time() + timeout
    next_req = 0.0
    while time.time() < end:
        now = time.time()
        if now >= next_req:
            _mission_request_list_send(master)
            next_req = now + 0.8
        msg = master.recv_match(type="MISSION_COUNT", blocking=True, timeout=0.4)
        if msg is None:
            continue
        return int(getattr(msg, "count", -1))
    return None


def _mission_item_int_send(master, item: dict) -> None:
    try:
        master.mav.mission_item_int_send(
            master.target_system,
            master.target_component,
            item["seq"],
            item["frame_int"],
            item["command"],
            item["current"],
            item["autocontinue"],
            item["p1"],
            item["p2"],
            item["p3"],
            item["p4"],
            item["lat_i"],
            item["lon_i"],
            item["alt"],
            mavutil.mavlink.MAV_MISSION_TYPE_MISSION,
        )
    except TypeError:
        master.mav.mission_item_int_send(
            master.target_system,
            master.target_component,
            item["seq"],
            item["frame_int"],
            item["command"],
            item["current"],
            item["autocontinue"],
            item["p1"],
            item["p2"],
            item["p3"],
            item["p4"],
            item["lat_i"],
            item["lon_i"],
            item["alt"],
        )


def _mission_item_send(master, item: dict) -> None:
    master.mav.mission_item_send(
        master.target_system,
        master.target_component,
        item["seq"],
        item["frame_float"],
        item["command"],
        item["current"],
        item["autocontinue"],
        item["p1"],
        item["p2"],
        item["p3"],
        item["p4"],
        item["lat_f"],
        item["lon_f"],
        item["alt"],
    )


def _build_demo_mission(lat: float, lon: float, rel_alt: float) -> List[dict]:
    # Create a small triangle mission: takeoff then two waypoints.
    d_north = 30.0
    d_east = 30.0
    dlat = d_north / 111111.0
    dlon = d_east / (111111.0 * max(0.1, math.cos(math.radians(lat))))

    points = [
        (lat, lon, mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 1),
        (lat + dlat, lon, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0),
        (lat + dlat, lon + dlon, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0),
    ]

    out: List[dict] = []
    for seq, (la, lo, cmd, current) in enumerate(points):
        out.append(
            {
                "seq": seq,
                "frame_int": mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
                "frame_float": mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
                "command": cmd,
                "current": current,
                "autocontinue": 1,
                "p1": 0.0,
                "p2": 0.0,
                "p3": 0.0,
                "p4": 0.0,
                "lat_i": int(la * 1e7),
                "lon_i": int(lo * 1e7),
                "lat_f": float(la),
                "lon_f": float(lo),
                "alt": float(rel_alt),
            }
        )
    return out


def _to_int_frame(frame: int) -> int:
    mapping = {
        int(mavutil.mavlink.MAV_FRAME_GLOBAL): int(mavutil.mavlink.MAV_FRAME_GLOBAL_INT),
        int(mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT): int(
            mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT
        ),
        int(mavutil.mavlink.MAV_FRAME_GLOBAL_TERRAIN_ALT): int(
            mavutil.mavlink.MAV_FRAME_GLOBAL_TERRAIN_ALT_INT
        ),
    }
    return int(mapping.get(int(frame), int(frame)))


def _parse_qgc_wpl_110(path: pathlib.Path) -> List[dict]:
    lines = path.read_text(encoding="utf-8", errors="ignore").splitlines()
    if not lines:
        raise RuntimeError(f"Mission file is empty: {path}")
    if not lines[0].strip().startswith("QGC WPL 110"):
        raise RuntimeError(f"Unsupported mission header in {path}: {lines[0]!r}")

    items: List[dict] = []
    for idx, raw in enumerate(lines[1:], start=2):
        line = raw.strip()
        if not line:
            continue
        parts = line.split()
        if len(parts) < 12:
            raise RuntimeError(f"Invalid waypoint line {idx} in {path}: {line!r}")

        seq = int(parts[0])
        current = int(parts[1])
        frame = int(parts[2])
        command = int(parts[3])
        p1 = float(parts[4])
        p2 = float(parts[5])
        p3 = float(parts[6])
        p4 = float(parts[7])
        lat_f = float(parts[8])
        lon_f = float(parts[9])
        alt = float(parts[10])
        autocontinue = int(parts[11])

        items.append(
            {
                "seq": seq,
                "frame_int": _to_int_frame(frame),
                "frame_float": frame,
                "command": command,
                "current": current,
                "autocontinue": autocontinue,
                "p1": p1,
                "p2": p2,
                "p3": p3,
                "p4": p4,
                "lat_i": int(lat_f * 1e7),
                "lon_i": int(lon_f * 1e7),
                "lat_f": lat_f,
                "lon_f": lon_f,
                "alt": alt,
            }
        )

    if not items:
        raise RuntimeError(f"No mission waypoints parsed from {path}")

    # Enforce contiguous sequence order expected by upload handler.
    items = sorted(items, key=lambda x: x["seq"])
    for new_seq, item in enumerate(items):
        item["seq"] = new_seq
    return items


def _upload_mission(master, items: List[dict], timeout: float = 30.0) -> None:
    debug_upload = os.getenv("ATT_MISSION_DEBUG", "0") == "1"
    # Drain stale mission ACKs that may be left from prior operations.
    stale_end = time.time() + 1.0
    while time.time() < stale_end:
        stale = master.recv_match(type="MISSION_ACK", blocking=True, timeout=0.1)
        if stale is None:
            continue
        if debug_upload:
            print(f"[DEBUG] Dropping stale pre-clear MISSION_ACK type={getattr(stale, 'type', None)}")

    _mission_clear_all(master)
    # CLEAR_ALL may emit its own MISSION_ACK; discard before upload transaction.
    clear_end = time.time() + 1.0
    while time.time() < clear_end:
        clear_ack = master.recv_match(type="MISSION_ACK", blocking=True, timeout=0.1)
        if clear_ack is None:
            continue
        if debug_upload:
            print(
                f"[DEBUG] Dropping clear-all MISSION_ACK type={getattr(clear_ack, 'type', None)}"
            )

    _mission_count_send(master, len(items))
    sent = set()
    end = time.time() + timeout
    while len(sent) < len(items) and time.time() < end:
        req = master.recv_match(
            type=["MISSION_REQUEST_INT", "MISSION_REQUEST"], blocking=True, timeout=1.0
        )
        if req is None:
            continue
        seq = int(getattr(req, "seq", -1))
        if seq < 0 or seq >= len(items):
            continue
        if debug_upload:
            print(f"[DEBUG] Mission request {req.get_type()} seq={seq}")
        if req.get_type() == "MISSION_REQUEST_INT":
            _mission_item_int_send(master, items[seq])
        else:
            _mission_item_send(master, items[seq])
        sent.add(seq)

    if len(sent) < len(items):
        raise RuntimeError("Mission upload timed out waiting for all mission requests.")

    ack = master.recv_match(type="MISSION_ACK", blocking=True, timeout=8.0)
    if ack is None:
        raise RuntimeError("No MISSION_ACK received.")
    ack_type = int(getattr(ack, "type", -1))
    if debug_upload:
        print(f"[DEBUG] Mission ACK type={ack_type}")
        try:
            print(f"[DEBUG] Mission ACK payload={ack.to_dict()}")
        except Exception:
            pass
    if ack_type != int(mavutil.mavlink.MAV_MISSION_ACCEPTED):
        texts: List[str] = []
        end_text = time.time() + 2.0
        while time.time() < end_text:
            st = master.recv_match(type="STATUSTEXT", blocking=True, timeout=0.2)
            if st is None:
                continue
            txt = getattr(st, "text", "")
            if isinstance(txt, bytes):
                txt = txt.decode("utf-8", errors="ignore")
            txt = str(txt).strip()
            if txt:
                texts.append(txt)
        remote_count = _read_mission_count(master, timeout=3.0)
        if remote_count == len(items):
            print(
                "[WARN] Mission ACK was non-accepted, but vehicle reports expected "
                f"mission count ({remote_count}); continuing."
            )
            return
        extra = f" Last STATUSTEXT: {' | '.join(texts)}" if texts else ""
        raise RuntimeError(
            f"Mission upload rejected. ACK type={getattr(ack, 'type', None)}.{extra}"
        )


def _start_mission(master) -> None:
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_MISSION_START,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
    )


def _guided_takeoff(master, rel_alt_m: float, timeout: float = 30.0) -> None:
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        float(rel_alt_m),
    )
    end = time.time() + timeout
    while time.time() < end:
        msg = master.recv_match(type="GLOBAL_POSITION_INT", blocking=True, timeout=0.8)
        if msg is None:
            continue
        rel_alt = float(getattr(msg, "relative_alt", 0.0)) / 1000.0
        if rel_alt >= max(2.0, rel_alt_m * 0.6):
            return
    raise RuntimeError("GUIDED takeoff timeout.")


def _distance_m(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
    dlat = math.radians(lat2 - lat1)
    dlon = math.radians(lon2 - lon1)
    lat1r = math.radians(lat1)
    lat2r = math.radians(lat2)
    a = math.sin(dlat / 2.0) ** 2 + math.cos(lat1r) * math.cos(lat2r) * math.sin(dlon / 2.0) ** 2
    return 2.0 * 6371000.0 * math.asin(min(1.0, math.sqrt(max(0.0, a))))


def _wait_for_mission_progress(
    master,
    timeout: float,
    min_seq: int,
    min_move_m: float,
    start_lat: float | None,
    start_lon: float | None,
) -> bool:
    end = time.time() + timeout
    highest_seq = -1
    while time.time() < end:
        msg = master.recv_match(
            type=["MISSION_CURRENT", "GLOBAL_POSITION_INT", "STATUSTEXT"],
            blocking=True,
            timeout=0.8,
        )
        if msg is None:
            continue
        mtype = msg.get_type()
        if mtype == "MISSION_CURRENT":
            seq = int(getattr(msg, "seq", -1))
            highest_seq = max(highest_seq, seq)
            if highest_seq >= min_seq:
                print(f"[INFO] Mission progressed to sequence {highest_seq}.")
                return True
        if mtype == "GLOBAL_POSITION_INT" and start_lat is not None and start_lon is not None:
            lat = float(getattr(msg, "lat", 0)) / 1e7
            lon = float(getattr(msg, "lon", 0)) / 1e7
            if abs(lat) > 0.001 and abs(lon) > 0.001:
                moved = _distance_m(start_lat, start_lon, lat, lon)
                if moved >= min_move_m:
                    print(f"[INFO] Vehicle moved {moved:.1f} m along mission path.")
                    return True
    return False


def _sleep_with_heartbeat(master, seconds: float) -> None:
    end = time.time() + max(0.0, seconds)
    while time.time() < end:
        remaining = max(0.0, end - time.time())
        master.recv_match(type=["HEARTBEAT", "STATUSTEXT"], blocking=True, timeout=min(1.0, remaining))


def _read_rawev_value(resp_path: pathlib.Path, key: str) -> str:
    obj = json.loads(resp_path.read_text(encoding="utf-8"))
    payload = obj.get("PAYLOAD", [])
    if not isinstance(payload, list) or not payload:
        raise RuntimeError(f"Unexpected response shape in {resp_path}")
    raw_obj = payload[0]
    if not isinstance(raw_obj, dict):
        raise RuntimeError("Expected PAYLOAD[0] object with RawEv list.")
    raw_ev = raw_obj.get("RawEv", [])
    if not isinstance(raw_ev, list):
        raise RuntimeError("Expected RawEv list in response.")
    prefix = key + "="
    for item in raw_ev:
        if isinstance(item, str) and item.startswith(prefix):
            return item[len(prefix) :]
    raise RuntimeError(f"Could not find '{key}' in RawEv.")


def _resolve_expected_hash(
    measured: str,
    force_fail: bool,
    expected_hash: str | None,
    expected_hash_file: pathlib.Path | None,
) -> tuple[str, str]:
    expected = None
    expected_reason = ""
    if expected_hash:
        expected = expected_hash.strip()
        expected_reason = "provided explicit expected hash"
    elif expected_hash_file and expected_hash_file.exists():
        txt = expected_hash_file.read_text(encoding="utf-8", errors="ignore").strip()
        if txt:
            expected = txt.splitlines()[0].strip()
            expected_reason = f"baseline file {expected_hash_file}"
    if expected is None:
        if force_fail:
            expected = "0" * 64
            expected_reason = "forced-fail demo mode"
        else:
            expected = measured
            expected_reason = "pass-through measured hash"
    return expected, expected_reason


def _run_one_attestation_and_verify(
    root: pathlib.Path,
    label: str,
    run_script_name: str,
    response_name: str,
    key_name: str,
    force_fail: bool,
    expected_hash: str | None,
    expected_hash_file: pathlib.Path | None,
) -> int:
    run_script = root / "scripts" / run_script_name
    verify_script = root / "scripts" / "verify_raw_ev_measurement.py"
    resp_path = root / "outputs" / response_name

    run_res = _run([str(run_script)], cwd=root)
    sys.stdout.write(run_res.stdout)
    sys.stderr.write(run_res.stderr)
    if run_res.returncode != 0:
        raise RuntimeError(f"Attestation run script failed with code {run_res.returncode}")

    measured = _read_rawev_value(resp_path, key_name)
    expected, expected_reason = _resolve_expected_hash(
        measured=measured,
        force_fail=force_fail,
        expected_hash=expected_hash,
        expected_hash_file=expected_hash_file,
    )
    print(f"[INFO] {label} expected hash source: {expected_reason}")

    ver_res = _run(
        [
            sys.executable,
            str(verify_script),
            "--response",
            str(resp_path),
            "--key",
            key_name,
            "--expected",
            expected,
        ],
        cwd=root,
    )
    sys.stdout.write(ver_res.stdout)
    sys.stderr.write(ver_res.stderr)
    return ver_res.returncode


def _run_attestation_and_verify(
    root: pathlib.Path,
    param_run_script_name: str,
    param_response_name: str,
    force_fail: bool,
    expected_param_hash: str | None,
    expected_param_hash_file: pathlib.Path | None,
    expected_mission_hash: str | None,
    expected_mission_hash_file: pathlib.Path | None,
    run_mission_attestation: bool,
) -> int:
    param_rc = _run_one_attestation_and_verify(
        root=root,
        label="Parameter appraisal",
        run_script_name=param_run_script_name,
        response_name=param_response_name,
        key_name="ardupilot_param_sha256",
        force_fail=force_fail,
        expected_hash=expected_param_hash,
        expected_hash_file=expected_param_hash_file,
    )
    if param_rc != 0:
        return param_rc

    if not run_mission_attestation:
        return 0

    mission_rc = _run_one_attestation_and_verify(
        root=root,
        label="Mission appraisal",
        run_script_name="run_mission_hash_demo.sh",
        response_name="mission_hash_demo.response.json",
        key_name="ardupilot_mission_sha256",
        force_fail=force_fail,
        expected_hash=expected_mission_hash,
        expected_hash_file=expected_mission_hash_file,
    )
    return mission_rc


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--master", default="udp:127.0.0.1:14550", help="MAVLink endpoint")
    parser.add_argument(
        "--heartbeat-timeout",
        type=float,
        default=45.0,
        help="Seconds to wait for first MAVLink heartbeat.",
    )
    parser.add_argument("--alt", type=float, default=20.0, help="Mission relative altitude (m)")
    parser.add_argument(
        "--mission-file",
        default="/home/michaeldoran/git/CRABWAVE/MP2/mission7.waypoints",
        help="QGC WPL 110 mission file; if missing, script uses generated demo mission.",
    )
    parser.add_argument(
        "--force-fail-attestation",
        action="store_true",
        default=True,
        help="Intentionally fail appraisal and trigger RTL (default true; ignored when baseline hash is provided).",
    )
    parser.add_argument(
        "--pass-attestation",
        action="store_true",
        help="Override and make appraisal pass; no RTL trigger demo.",
    )
    parser.add_argument(
        "--expected-hash",
        default=None,
        help="Expected parameter baseline hash for real appraisal (legacy alias).",
    )
    parser.add_argument(
        "--expected-hash-file",
        default=None,
        help="Parameter baseline hash file (legacy alias). Default: outputs/param_hash_baseline.sha256 when present.",
    )
    parser.add_argument(
        "--param-profile",
        choices=["selected", "all"],
        default="selected",
        help="Parameter appraisal profile: selected fields or all available parameters.",
    )
    parser.add_argument(
        "--expected-mission-hash",
        default=None,
        help="Expected mission baseline hash for real appraisal. If mismatch, attestation becomes INVALID.",
    )
    parser.add_argument(
        "--expected-mission-hash-file",
        default=None,
        help="Mission baseline hash file (first line). Default: outputs/mission_hash_baseline.sha256 when present.",
    )
    parser.add_argument(
        "--no-mission-attestation",
        action="store_true",
        help="Skip mission-waypoint appraisal and only evaluate parameter hash.",
    )
    parser.add_argument(
        "--skip-mission-upload",
        action="store_true",
        help="Assume the mission is already uploaded; do not re-run the upload sequence.",
    )
    parser.add_argument(
        "--skip-auto-flight",
        action="store_true",
        help="Skip automated GUIDED takeoff/AUTO mission start; expect manual control via MAVProxy.",
    )
    parser.add_argument("--attest-root", default=None, help="Path to ardupilot-attest root")
    parser.add_argument(
        "--no-disable-arming-check",
        action="store_true",
        help="Do not set ARMING_CHECK=0 before arming (stricter realism).",
    )
    parser.add_argument(
        "--pre-attestation-timeout",
        type=float,
        default=25.0,
        help="Seconds to let mission progress before attestation.",
    )
    parser.add_argument(
        "--pre-attestation-min-seq",
        type=int,
        default=1,
        help="Minimum mission sequence index to consider as progressed.",
    )
    parser.add_argument(
        "--pre-attestation-min-move-m",
        type=float,
        default=8.0,
        help="Minimum movement distance (m) to consider as progressed.",
    )
    parser.add_argument(
        "--periodic-attestation",
        action="store_true",
        help="Run repeated appraisal cycles instead of a single one-shot check.",
    )
    parser.add_argument(
        "--attestation-interval",
        type=float,
        default=20.0,
        help="Seconds between appraisal cycles when --periodic-attestation is enabled.",
    )
    parser.add_argument(
        "--attestation-cycles",
        type=int,
        default=0,
        help="Number of appraisal cycles in periodic mode (0 means run until invalid/failure).",
    )
    args = parser.parse_args()

    force_fail = False if args.pass_attestation else bool(args.force_fail_attestation)
    root = (
        pathlib.Path(args.attest_root).resolve()
        if args.attest_root
        else pathlib.Path(__file__).resolve().parents[1]
    )
    if args.param_profile == "all":
        param_run_script_name = "run_param_hash_all_demo.sh"
        param_response_name = "param_hash_all_demo.response.json"
        default_param_expected_file = root / "outputs" / "param_hash_all_baseline.sha256"
    else:
        param_run_script_name = "run_param_hash_demo.sh"
        param_response_name = "param_hash_demo.response.json"
        default_param_expected_file = root / "outputs" / "param_hash_baseline.sha256"

    expected_param_hash_file = (
        pathlib.Path(args.expected_hash_file).expanduser().resolve()
        if args.expected_hash_file
        else (default_param_expected_file if default_param_expected_file.exists() else None)
    )
    default_mission_expected_file = root / "outputs" / "mission_hash_baseline.sha256"
    expected_mission_hash_file = (
        pathlib.Path(args.expected_mission_hash_file).expanduser().resolve()
        if args.expected_mission_hash_file
        else (default_mission_expected_file if default_mission_expected_file.exists() else None)
    )

    print(f"[INFO] Connecting to vehicle: {args.master}")
    # Use a distinct GCS system ID to avoid clashing with MAVProxy's default (255).
    master = mavutil.mavlink_connection(args.master, source_system=250, source_component=1)
    _wait_heartbeat(master, timeout=args.heartbeat_timeout)
    print(
        f"[INFO] Heartbeat received from system={master.target_system} component={master.target_component}"
    )

    mission_file = pathlib.Path(args.mission_file).expanduser()
    if args.skip_mission_upload:
        print("[INFO] Skipping mission upload (manual mission assumed).")
    else:
        if mission_file.exists():
            mission = _parse_qgc_wpl_110(mission_file)
            print(f"[INFO] Uploading mission file {mission_file} with {len(mission)} items...")
        else:
            lat, lon = _get_position(master)
            mission = _build_demo_mission(lat, lon, args.alt)
            print(
                f"[WARN] Mission file not found ({mission_file}); using generated demo mission ({len(mission)} items)."
            )
        _upload_mission(master, mission)

    auto_started = False
    start_lat = start_lon = None
    if not args.skip_auto_flight:
        if not args.no_disable_arming_check:
            print("[INFO] Setting ARMING_CHECK=0 for SITL demo convenience...")
            try:
                _set_param(master, "ARMING_CHECK", 0)
            except Exception as exc:
                print(f"[WARN] Could not confirm ARMING_CHECK update: {exc}")

        print("[INFO] Switching to GUIDED, arming, and taking off...")
        _set_mode(master, "GUIDED")
        _arm(master)
        _guided_takeoff(master, rel_alt_m=args.alt)
        start_lat, start_lon = _get_position(master, timeout=8.0)

        print("[INFO] Attempting AUTO mission start after takeoff...")
        try:
            _set_mode(master, "AUTO")
            _start_mission(master)
            auto_started = True
            print(
                "[INFO] AUTO mission running; waiting for mission progress before attestation..."
            )
            progressed = _wait_for_mission_progress(
                master,
                timeout=args.pre_attestation_timeout,
                min_seq=args.pre_attestation_min_seq,
                min_move_m=args.pre_attestation_min_move_m,
                start_lat=start_lat,
                start_lon=start_lon,
            )
            if not progressed:
                print(
                    "[WARN] Did not observe mission progress signal before attestation timeout; proceeding."
                )
        except Exception as exc:
            print(f"[WARN] AUTO mission start failed: {exc}")
            print("[INFO] Falling back to GUIDED leg before attestation.")
            _set_mode(master, "GUIDED")
            if not master.motors_armed():
                print("[INFO] Vehicle disarmed during AUTO attempt; re-arming in GUIDED.")
                _arm(master)
            _guided_takeoff(master, rel_alt_m=args.alt)
            time.sleep(min(5.0, max(0.0, args.pre_attestation_timeout)))
    else:
        print("[INFO] Skipping guided takeoff/mission start; assume MAVProxy handles flight.")

    mode_desc = "periodic" if args.periodic_attestation else "one-shot"
    print(f"[INFO] Running {mode_desc} attestation/appraisal while mission is active...")
    cycle = 0
    while True:
        cycle += 1
        print(f"[INFO] Appraisal cycle {cycle} starting...")
        verify_rc = _run_attestation_and_verify(
            root=root,
            param_run_script_name=param_run_script_name,
            param_response_name=param_response_name,
            force_fail=force_fail,
            expected_param_hash=args.expected_hash,
            expected_param_hash_file=expected_param_hash_file,
            expected_mission_hash=args.expected_mission_hash,
            expected_mission_hash_file=expected_mission_hash_file,
            run_mission_attestation=not args.no_mission_attestation,
        )

        if verify_rc == 2:
            print("[INFO] Attestation INVALID -> commanding RTL.")
            _set_mode(master, "RTL")
            print("[RESULT] RTL engaged due to attestation failure.")
            return 0
        if verify_rc != 0:
            print(f"[ERROR] Verifier returned unexpected code {verify_rc}.")
            return 1

        if not args.periodic_attestation:
            if auto_started:
                print("[RESULT] Attestation VALID -> mission continues in AUTO.")
            else:
                if args.skip_auto_flight:
                    print("[RESULT] Attestation VALID -> vehicle under operator control.")
                else:
                    print("[RESULT] Attestation VALID -> vehicle remains in GUIDED demo leg.")
            return 0

        if args.attestation_cycles > 0 and cycle >= args.attestation_cycles:
            print(f"[RESULT] Completed {cycle} valid periodic appraisal cycles; mission continues.")
            return 0

        print(
            f"[INFO] Appraisal cycle {cycle} valid; waiting {args.attestation_interval:.1f}s before next cycle."
        )
        _sleep_with_heartbeat(master, args.attestation_interval)


if __name__ == "__main__":
    raise SystemExit(main())
    if args.periodic_attestation and args.attestation_interval <= 0:
        raise RuntimeError("--attestation-interval must be > 0 in periodic mode.")
    if args.attestation_cycles < 0:
        raise RuntimeError("--attestation-cycles must be >= 0.")
