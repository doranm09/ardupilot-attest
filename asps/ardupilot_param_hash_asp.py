#!/usr/bin/env python3
"""
ArduPilot parameter-measurement ASP.

Input (stdin): ASP_RUN request JSON
Output (stdout): ASP_RUN response JSON with RawEv entries, for example:
  {
    "TYPE":"RESPONSE",
    "ACTION":"ASP_RUN",
    "SUCCESS":true,
    "PAYLOAD":{"RawEv":["ardupilot_param_sha256=...","ardupilot_param_count=..."]}
  }

ASP_ARGS contract (all optional):
  master: MAVLink endpoint string (default: "udp:127.0.0.1:14550")
  timeout_sec: total timeout in seconds (default: 20)
  param_names: list of explicit parameter names to query
  param_prefix: prefix filter used when param_names is omitted
  exclude_names: list of exact parameter names to exclude from hash input
  exclude_prefixes: list of parameter prefixes to exclude from hash input
"""

from __future__ import annotations

import hashlib
import json
import sys
import time
from typing import Dict, List, Set, Tuple


def _err(msg: str, code: int = 1) -> None:
    print(msg, file=sys.stderr)
    raise SystemExit(code)


def _normalize_param_id(param_id_obj) -> str:
    if isinstance(param_id_obj, bytes):
        text = param_id_obj.decode("utf-8", errors="ignore")
    else:
        text = str(param_id_obj)
    return text.split("\x00", 1)[0].strip().upper()


def _normalize_param_name(name: str) -> str:
    return name.split("\x00", 1)[0].strip().upper()


def _parse_string_list(value: object, field_name: str) -> List[str]:
    if not isinstance(value, list):
        _err(f"{field_name} must be a JSON array of strings.")
    out: List[str] = []
    for item in value:
        if not isinstance(item, str):
            _err(f"{field_name} entries must be strings.")
        out.append(_normalize_param_name(item))
    return out


def _parse_args(req: dict) -> Tuple[str, float, List[str], str, List[str], List[str]]:
    asp_args = req.get("ASP_ARGS", {})
    if not isinstance(asp_args, dict):
        _err("ASP_ARGS must be a JSON object.")

    master = asp_args.get("master", "udp:127.0.0.1:14550")
    timeout_sec_raw = asp_args.get("timeout_sec", 20)
    param_names_raw = asp_args.get("param_names", [])
    param_prefix = asp_args.get("param_prefix", "")
    exclude_names_raw = asp_args.get("exclude_names", [])
    exclude_prefixes_raw = asp_args.get("exclude_prefixes", [])

    try:
        timeout_sec = float(timeout_sec_raw)
    except Exception as exc:  # pragma: no cover
        _err(f"Invalid timeout_sec value: {timeout_sec_raw!r} ({exc})")

    param_names = _parse_string_list(param_names_raw, "param_names")
    exclude_names = _parse_string_list(exclude_names_raw, "exclude_names")
    exclude_prefixes = _parse_string_list(exclude_prefixes_raw, "exclude_prefixes")

    if not isinstance(param_prefix, str):
        _err("param_prefix must be a string.")

    if timeout_sec <= 0:
        _err("timeout_sec must be > 0.")

    return str(master), timeout_sec, param_names, param_prefix, exclude_names, exclude_prefixes


def _fetch_named_params(
    master_conn, target_sys: int, target_comp: int, names: List[str], deadline: float
) -> Dict[str, float]:
    wanted: Set[str] = set(names)
    found: Dict[str, float] = {}
    next_retry = 0.0

    while time.time() < deadline and wanted - set(found.keys()):
        now = time.time()
        if now >= next_retry:
            # Re-send requests for still-missing names; some endpoints are lossy.
            for name in sorted(wanted - set(found.keys())):
                master_conn.mav.param_request_read_send(
                    target_sys, target_comp, name.encode("utf-8"), -1
                )
            next_retry = now + 1.0

        msg = master_conn.recv_match(type="PARAM_VALUE", blocking=True, timeout=0.5)
        if msg is None:
            continue
        key = _normalize_param_id(msg.param_id)
        if key in wanted:
            found[key] = float(msg.param_value)

    return found


def _fetch_all_params(master_conn, target_sys: int, target_comp: int, deadline: float) -> Dict[str, float]:
    master_conn.mav.param_request_list_send(target_sys, target_comp)
    params: Dict[str, float] = {}
    expected_count = None

    while time.time() < deadline:
        msg = master_conn.recv_match(type="PARAM_VALUE", blocking=True, timeout=0.5)
        if msg is None:
            continue

        key = _normalize_param_id(msg.param_id)
        params[key] = float(msg.param_value)

        try:
            c = int(msg.param_count)
            if c > 0:
                expected_count = c
        except Exception:
            pass

        if expected_count is not None and len(params) >= expected_count:
            break

    return params


def _canonical_lines(params: Dict[str, float]) -> List[str]:
    lines: List[str] = []
    for k in sorted(params.keys()):
        v = params[k]
        lines.append(f"{k}={format(v, '.9g')}")
    return lines


def _is_excluded(name: str, exclude_names: Set[str], exclude_prefixes: Tuple[str, ...]) -> bool:
    if name in exclude_names:
        return True
    if exclude_prefixes and any(name.startswith(prefix) for prefix in exclude_prefixes):
        return True
    return False


def _apply_exclusions(
    params: Dict[str, float], exclude_names: Set[str], exclude_prefixes: Tuple[str, ...]
) -> Dict[str, float]:
    if not exclude_names and not exclude_prefixes:
        return params
    return {
        k: v
        for k, v in params.items()
        if not _is_excluded(k, exclude_names, exclude_prefixes)
    }


def _hint_candidates(missing: List[str], all_params: Dict[str, float]) -> List[str]:
    if not all_params:
        return []
    keys = sorted(all_params.keys())
    hints: List[str] = []
    for miss in missing:
        tokens = [t for t in miss.split("_") if len(t) >= 3]
        for k in keys:
            if any(t in k for t in tokens):
                hints.append(k)
            if len(hints) >= 8:
                return hints
    return hints


def main() -> None:
    req_text = sys.stdin.read()
    if not req_text.strip():
        _err("No ASP request JSON on stdin.")

    req = json.loads(req_text)
    (
        master_endpoint,
        timeout_sec,
        param_names,
        param_prefix,
        exclude_names,
        exclude_prefixes,
    ) = _parse_args(req)
    exclude_name_set = set(exclude_names)
    exclude_prefixes_tuple = tuple(exclude_prefixes)

    if param_names:
        excluded_requested = [
            name
            for name in param_names
            if _is_excluded(name, exclude_name_set, exclude_prefixes_tuple)
        ]
        if excluded_requested:
            _err(
                "param_names contains excluded entries: "
                f"{sorted(set(excluded_requested))}. "
                "Remove from param_names or adjust exclude_names/exclude_prefixes."
            )

    try:
        from pymavlink import mavutil  # type: ignore
    except Exception as exc:
        _err(
            "pymavlink is not installed in this Python environment. "
            "Install it before using ardupilot_param_hash_asp.py. "
            f"Import error: {exc}"
        )

    t_start = time.time()
    deadline = t_start + timeout_sec

    conn = mavutil.mavlink_connection(master_endpoint, source_system=255)
    hb_timeout = max(1.0, min(10.0, timeout_sec))
    hb = conn.wait_heartbeat(timeout=hb_timeout)
    if hb is None:
        _err(f"No MAVLink heartbeat received from {master_endpoint} within {hb_timeout}s.")

    target_sys = int(conn.target_system or 1)
    target_comp = int(conn.target_component or 1)

    if param_names:
        named_deadline = t_start + (timeout_sec * 0.6)
        measured = _fetch_named_params(conn, target_sys, target_comp, param_names, named_deadline)
        missing = [n for n in param_names if n not in measured]
        all_params: Dict[str, float] = {}
        if missing and time.time() < deadline:
            all_params = _fetch_all_params(conn, target_sys, target_comp, deadline)
            for n in list(missing):
                if n in all_params:
                    measured[n] = all_params[n]
            missing = [n for n in param_names if n not in measured]
        if missing:
            hints = _hint_candidates(missing, all_params)
            hint_text = f" Similar available params: {hints}" if hints else ""
            _err(f"Did not receive requested params before timeout: {missing}.{hint_text}")
    else:
        all_params = _fetch_all_params(conn, target_sys, target_comp, deadline)
        if not all_params:
            _err(f"No PARAM_VALUE messages received from {master_endpoint}.")
        if param_prefix:
            measured = {k: v for k, v in all_params.items() if k.startswith(param_prefix)}
        else:
            measured = all_params
        if not measured:
            _err(f"No parameters matched param_prefix={param_prefix!r}.")

    measured = _apply_exclusions(measured, exclude_name_set, exclude_prefixes_tuple)
    if not measured:
        _err(
            "All measured parameters were excluded. "
            "Adjust exclude_names/exclude_prefixes filters."
        )

    lines = _canonical_lines(measured)
    digest = hashlib.sha256("\n".join(lines).encode("utf-8")).hexdigest()

    payload = {
        "RawEv": [
            f"ardupilot_param_sha256={digest}",
            f"ardupilot_param_count={len(lines)}",
            f"ardupilot_param_source={master_endpoint}",
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
