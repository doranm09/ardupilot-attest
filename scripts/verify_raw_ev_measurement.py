#!/usr/bin/env python3
"""
Verify a RawEv measurement entry in a CVM RUN response.

Examples:
  verify_raw_ev_measurement.py --response outputs/fw_hash_demo.response.json \
    --key fw_sha256 --expected demo

  verify_raw_ev_measurement.py --response outputs/param_hash_demo.response.json \
    --key ardupilot_param_sha256 --baseline-file outputs/param_hash.baseline
"""

from __future__ import annotations

import argparse
import json
import pathlib
import sys
from typing import Any, Dict, List


def _load_response(path: pathlib.Path) -> Dict[str, Any]:
    text = path.read_text(encoding="utf-8", errors="ignore").strip()
    if not text:
        raise ValueError(f"Empty response file: {path}")

    # Try full-file JSON first
    try:
        obj = json.loads(text)
        if isinstance(obj, dict):
            return obj
    except Exception:
        pass

    # Fallback: parse last JSON object line
    last_obj = None
    for line in text.splitlines():
        line = line.strip()
        if not line:
            continue
        try:
            obj = json.loads(line)
        except Exception:
            continue
        if isinstance(obj, dict):
            last_obj = obj
    if last_obj is None:
        raise ValueError(f"No JSON object found in file: {path}")
    return last_obj


def _extract_raw_ev(resp: Dict[str, Any]) -> List[str]:
    payload = resp.get("PAYLOAD")
    if not isinstance(payload, list) or len(payload) < 1:
        raise ValueError("Unexpected PAYLOAD shape; expected list with RawEv at index 0.")
    raw_ev_obj = payload[0]
    if not isinstance(raw_ev_obj, dict):
        raise ValueError("Unexpected RawEv payload element type.")
    raw_ev = raw_ev_obj.get("RawEv")
    if not isinstance(raw_ev, list):
        raise ValueError("RawEv not present in response payload.")
    out: List[str] = []
    for v in raw_ev:
        if isinstance(v, str):
            out.append(v)
    return out


def _extract_value(raw_ev: List[str], key: str) -> str:
    prefix = f"{key}="
    for item in raw_ev:
        if item.startswith(prefix):
            return item[len(prefix):]
    raise ValueError(f"Measurement key '{key}' not found in RawEv.")


def _read_expected(expected: str | None, baseline_file: pathlib.Path | None) -> str:
    if expected is not None:
        return expected.strip()
    if baseline_file is None:
        raise ValueError("Either --expected or --baseline-file must be provided.")
    text = baseline_file.read_text(encoding="utf-8", errors="ignore").strip()
    if not text:
        raise ValueError(f"Empty baseline file: {baseline_file}")
    # accept either `key=value` or plain value
    if "=" in text:
        return text.split("=", 1)[1].strip()
    return text


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--response", required=True, help="Path to CVM response JSON/log file")
    parser.add_argument("--key", required=True, help="RawEv key (without '=')")
    parser.add_argument("--expected", help="Expected measurement value")
    parser.add_argument("--baseline-file", help="File containing expected value")
    args = parser.parse_args()

    try:
        resp = _load_response(pathlib.Path(args.response))
        raw_ev = _extract_raw_ev(resp)
        measured = _extract_value(raw_ev, args.key)
        expected = _read_expected(
            expected=args.expected,
            baseline_file=pathlib.Path(args.baseline_file) if args.baseline_file else None,
        )
    except Exception as exc:
        print(f"VERIFY_ERROR: {exc}", file=sys.stderr)
        return 1

    if measured == expected:
        print(f"VALID: {args.key} matched expected value '{expected}'.")
        return 0

    print(f"INVALID: {args.key} measured='{measured}' expected='{expected}'.")
    return 2


if __name__ == "__main__":
    raise SystemExit(main())

