#!/usr/bin/env python3
import json
import pathlib
import sys


def main() -> int:
    if len(sys.argv) != 3:
        print("Usage: extract_last_json.py <input.log> <output.json>", file=sys.stderr)
        return 2

    in_path = pathlib.Path(sys.argv[1])
    out_path = pathlib.Path(sys.argv[2])
    if not in_path.exists():
        print(f"Input file not found: {in_path}", file=sys.stderr)
        return 2

    last_obj = None
    for line in in_path.read_text(encoding="utf-8", errors="ignore").splitlines():
        text = line.strip()
        if not text:
            continue
        try:
            obj = json.loads(text)
        except Exception:
            continue
        if isinstance(obj, dict):
            last_obj = obj

    if last_obj is None:
        print(f"No JSON object found in log: {in_path}", file=sys.stderr)
        return 1

    out_path.write_text(json.dumps(last_obj, indent=2) + "\n", encoding="utf-8")
    print(f"Wrote response JSON: {out_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

