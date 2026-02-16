#!/usr/bin/env python3
"""
ArduPilot mission-waypoint measurement ASP.

Input (stdin): ASP_RUN request JSON
Output (stdout): ASP_RUN response JSON with RawEv entries:
  - ardupilot_mission_sha256=<digest>
  - ardupilot_mission_count=<count>
  - ardupilot_mission_source=<master endpoint>
"""

from __future__ import annotations

import hashlib
import json
import sys
import time
from typing import Dict, List, Tuple


def _err(msg: str, code: int = 1) -> None:
    print(msg, file=sys.stderr)
    raise SystemExit(code)


def _parse_args(req: dict) -> Tuple[str, float]:
    asp_args = req.get("ASP_ARGS", {})
    if not isinstance(asp_args, dict):
        _err("ASP_ARGS must be a JSON object.")

    master = str(asp_args.get("master", "udp:127.0.0.1:14550"))
    timeout_raw = asp_args.get("timeout_sec", 20)
    try:
        timeout_sec = float(timeout_raw)
    except Exception as exc:
        _err(f"Invalid timeout_sec value: {timeout_raw!r} ({exc})")

    if timeout_sec <= 0:
        _err("timeout_sec must be > 0.")

    return master, timeout_sec


def _fmt_num(v) -> str:
    return format(float(v), ".9g")


def _canonical_line(
    seq: int,
    frame: int,
    command: int,
    autocontinue: int,
    p1,
    p2,
    p3,
    p4,
    lat,
    lon,
    alt,
) -> str:
    return (
        # Intentionally ignore MISSION_ITEM.current. It is execution state (which item is "current")
        # and will change when switching to AUTO / starting the mission, even if the mission plan
        # itself is unchanged.
        f"{seq}|{frame}|{command}|{autocontinue}|"
        f"{_fmt_num(p1)}|{_fmt_num(p2)}|{_fmt_num(p3)}|{_fmt_num(p4)}|"
        f"{_fmt_num(lat)}|{_fmt_num(lon)}|{_fmt_num(alt)}"
    )


def _request_mission_list(conn, target_sys: int, target_comp: int) -> None:
    from pymavlink import mavutil  # type: ignore

    try:
        conn.mav.mission_request_list_send(
            target_sys,
            target_comp,
            mavutil.mavlink.MAV_MISSION_TYPE_MISSION,
        )
    except TypeError:
        conn.mav.mission_request_list_send(target_sys, target_comp)


def _request_mission_item(conn, target_sys: int, target_comp: int, seq: int) -> None:
    from pymavlink import mavutil  # type: ignore

    try:
        conn.mav.mission_request_int_send(
            target_sys,
            target_comp,
            seq,
            mavutil.mavlink.MAV_MISSION_TYPE_MISSION,
        )
    except TypeError:
        try:
            conn.mav.mission_request_int_send(target_sys, target_comp, seq)
        except Exception:
            conn.mav.mission_request_send(target_sys, target_comp, seq)


def _extract_item(msg) -> Tuple[int, str]:
    mtype = msg.get_type()
    seq = int(getattr(msg, "seq", -1))
    frame = int(getattr(msg, "frame", 0))
    command = int(getattr(msg, "command", 0))
    autocontinue = int(getattr(msg, "autocontinue", 0))
    p1 = float(getattr(msg, "param1", 0.0))
    p2 = float(getattr(msg, "param2", 0.0))
    p3 = float(getattr(msg, "param3", 0.0))
    p4 = float(getattr(msg, "param4", 0.0))

    if mtype == "MISSION_ITEM_INT":
        lat = float(getattr(msg, "x", 0)) / 1e7
        lon = float(getattr(msg, "y", 0)) / 1e7
        alt = float(getattr(msg, "z", 0.0))
    else:
        lat = float(getattr(msg, "x", 0.0))
        lon = float(getattr(msg, "y", 0.0))
        alt = float(getattr(msg, "z", 0.0))

    # MAV_CMD_NAV_TAKEOFF (22) may have its lat/lon rewritten by ArduPilot when the
    # mission is started (AUTO), even though the plan is semantically unchanged.
    # Normalizing takeoff position keeps the mission "plan hash" stable across takeoff.
    if command == 22:
        lat = 0.0
        lon = 0.0

    return seq, _canonical_line(seq, frame, command, autocontinue, p1, p2, p3, p4, lat, lon, alt)


def _fetch_mission_lines(conn, target_sys: int, target_comp: int, timeout_sec: float) -> List[str]:
    from pymavlink import mavutil  # type: ignore

    deadline = time.time() + timeout_sec
    count = None
    next_list_req = 0.0

    while time.time() < deadline and count is None:
        now = time.time()
        if now >= next_list_req:
            _request_mission_list(conn, target_sys, target_comp)
            next_list_req = now + 0.8
        msg = conn.recv_match(type="MISSION_COUNT", blocking=True, timeout=0.5)
        if msg is None:
            continue
        mtype = int(getattr(msg, "mission_type", mavutil.mavlink.MAV_MISSION_TYPE_MISSION))
        if mtype != int(mavutil.mavlink.MAV_MISSION_TYPE_MISSION):
            continue
        count = int(getattr(msg, "count", -1))

    if count is None or count < 0:
        _err("Failed to receive mission count before timeout.")

    if count == 0:
        return []

    lines_by_seq: Dict[int, str] = {}
    next_req_time = 0.0
    req_seq = 0

    while time.time() < deadline and len(lines_by_seq) < count:
        now = time.time()
        if now >= next_req_time:
            while req_seq < count and req_seq in lines_by_seq:
                req_seq += 1
            if req_seq < count:
                _request_mission_item(conn, target_sys, target_comp, req_seq)
            next_req_time = now + 0.35

        msg = conn.recv_match(
            type=["MISSION_ITEM_INT", "MISSION_ITEM", "MISSION_ACK"],
            blocking=True,
            timeout=0.4,
        )
        if msg is None:
            continue

        mtype = msg.get_type()
        if mtype in ("MISSION_ITEM_INT", "MISSION_ITEM"):
            seq, line = _extract_item(msg)
            if 0 <= seq < count:
                lines_by_seq[seq] = line
        elif mtype == "MISSION_ACK":
            ack_type = int(getattr(msg, "type", -1))
            if ack_type not in (
                int(mavutil.mavlink.MAV_MISSION_ACCEPTED),
                int(mavutil.mavlink.MAV_MISSION_OPERATION_CANCELLED),
            ):
                # Continue collecting; some stacks emit non-fatal ACK noise.
                pass

    if len(lines_by_seq) < count:
        missing = [str(i) for i in range(count) if i not in lines_by_seq]
        _err(f"Mission item fetch timed out. Missing seq: {', '.join(missing)}")

    return [lines_by_seq[i] for i in sorted(lines_by_seq.keys())]


def main() -> None:
    req_text = sys.stdin.read()
    if not req_text.strip():
        _err("No ASP request JSON on stdin.")

    req = json.loads(req_text)
    master_endpoint, timeout_sec = _parse_args(req)

    try:
        from pymavlink import mavutil  # type: ignore
    except Exception as exc:
        _err(
            "pymavlink is not installed in this Python environment. "
            "Install it before using ardupilot_mission_hash_asp.py. "
            f"Import error: {exc}"
        )

    conn = mavutil.mavlink_connection(master_endpoint, source_system=254)
    hb_timeout = max(1.0, min(12.0, timeout_sec))
    hb = conn.wait_heartbeat(timeout=hb_timeout)
    if hb is None:
        _err(f"No MAVLink heartbeat received from {master_endpoint} within {hb_timeout}s.")

    target_sys = int(conn.target_system or 1)
    target_comp = int(conn.target_component or 1)

    lines = _fetch_mission_lines(conn, target_sys, target_comp, timeout_sec)
    digest = hashlib.sha256("\n".join(lines).encode("utf-8")).hexdigest()

    payload = {
        "RawEv": [
            f"ardupilot_mission_sha256={digest}",
            f"ardupilot_mission_count={len(lines)}",
            f"ardupilot_mission_source={master_endpoint}",
        ]
    }

    out = {
        "TYPE": "RESPONSE",
        "ACTION": "ASP_RUN",
        "SUCCESS": True,
        "PAYLOAD": payload,
    }
    print(json.dumps(out, separators=(",", ":")))


if __name__ == "__main__":
    main()
