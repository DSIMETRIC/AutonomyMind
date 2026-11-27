#!/usr/bin/env python3
"""Replay the synthetic rosbag into the compiled Decision E2E runner."""

from __future__ import annotations

import sqlite3
import struct
import subprocess
import sys
from pathlib import Path


def decode_cdr_string(blob: bytes) -> str:
    if len(blob) < 8:
        raise ValueError("CDR payload too short")
    length = struct.unpack("<I", blob[4:8])[0]
    if length < 1 or 8 + length > len(blob):
        raise ValueError("invalid CDR string length")
    raw = blob[8:8 + length]
    if raw[-1] != 0:
        raise ValueError("CDR string missing terminator")
    return raw[:-1].decode("utf-8")


def main() -> int:
    if len(sys.argv) != 5:
        print(
            "usage: replay_synthetic_rosbag.py BAG_DB RUNNER CONFIG OUTPUT_JSON",
            file=sys.stderr,
        )
        return 2

    bag_db = Path(sys.argv[1]).resolve()
    runner = Path(sys.argv[2]).resolve()
    config = Path(sys.argv[3]).resolve()
    output_json = Path(sys.argv[4]).resolve()

    conn = sqlite3.connect(bag_db)
    rows = conn.execute(
        "SELECT timestamp,data FROM messages ORDER BY timestamp ASC"
    ).fetchall()
    conn.close()

    if not rows:
        raise RuntimeError("synthetic bag contains no messages")

    csv_payload = "\n".join(decode_cdr_string(data) for _, data in rows) + "\n"

    result = subprocess.run(
        [str(runner), str(config), str(output_json)],
        input=csv_payload,
        text=True,
        capture_output=True,
        check=False,
    )
    sys.stdout.write(result.stdout)
    sys.stderr.write(result.stderr)
    if result.returncode != 0:
        return result.returncode

    if not output_json.exists() or output_json.stat().st_size == 0:
        raise RuntimeError("Decision replay did not produce dashboard evidence")

    print(f"[PASS] replayed {len(rows)} rosbag messages through Decision")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
