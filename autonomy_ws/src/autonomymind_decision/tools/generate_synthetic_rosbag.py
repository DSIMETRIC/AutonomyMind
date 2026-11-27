#!/usr/bin/env python3
"""Create a deterministic rosbag2 SQLite dataset for Decision E2E testing.

The bag stores std_msgs/msg/String messages encoded as CDR. Each string contains
one CSV Decision input row. This keeps the bag consumable by rosbag2 tooling
while avoiding a dependency on project-specific ROS message packages.
"""

from __future__ import annotations

import json
import sqlite3
import struct
import sys
from pathlib import Path

TOPIC = "/autonomymind/synthetic/decision_input"
TYPE = "std_msgs/msg/String"


def cdr_string(text: str) -> bytes:
    """Encode std_msgs/msg/String using little-endian CDR."""
    payload = text.encode("utf-8") + b"\x00"
    # CDR encapsulation header 0x0001 = CDR little endian.
    return b"\x00\x01\x00\x00" + struct.pack("<I", len(payload)) + payload


def scenario_rows() -> list[dict]:
    """Return a deterministic temporal scenario exercising Decision and Full RTA."""
    rows = []
    dt = 0.2
    for index in range(101):
        t = round(index * dt, 6)

        row = {
            "timestamp_s": t,
            "armed": 1,
            "nav_a_valid": 1,
            "nav_b_valid": 1,
            "gnss_valid": 1,
            "vio_valid": 1,
            "nav_x_m": 5.0 * t,
            "nav_y_m": 0.0,
            "nav_z_m": 100.0,
            "heading_deg": 0.0,
            "vx_mps": 5.0,
            "vy_mps": 0.0,
            "vz_mps": 0.0,
            "threat_count": 0,
            "threat_rel_x_m": 0.0,
            "threat_rel_y_m": 0.0,
            "threat_rel_z_m": 0.0,
            "threat_vx_mps": 0.0,
            "threat_vy_mps": 0.0,
            "threat_vz_mps": 0.0,
        }

        # 4-8 s: GPS denied but VIO remains available.
        if 4.0 <= t < 8.0:
            row["gnss_valid"] = 0

        # 8-14 s: crossing traffic. Geometry is chosen so the no-action path
        # violates 15 m separation while at least one assured candidate survives.
        if 8.0 <= t < 14.0:
            elapsed = t - 8.0
            row["threat_count"] = 1
            row["threat_rel_x_m"] = 40.0 - 10.0 * elapsed
            row["threat_rel_y_m"] = 10.0
            row["threat_rel_z_m"] = 0.0
            row["threat_vx_mps"] = -5.0
            row["threat_vy_mps"] = 0.0
            row["threat_vz_mps"] = 0.0

        # 14-18 s: mission path offset to exercise rejoin after traffic clears.
        if 14.0 <= t < 18.0:
            row["nav_y_m"] = 6.0

        rows.append(row)
    return rows


FIELDS = [
    "timestamp_s", "armed", "nav_a_valid", "nav_b_valid", "gnss_valid",
    "vio_valid", "nav_x_m", "nav_y_m", "nav_z_m", "heading_deg",
    "vx_mps", "vy_mps", "vz_mps", "threat_count",
    "threat_rel_x_m", "threat_rel_y_m", "threat_rel_z_m",
    "threat_vx_mps", "threat_vy_mps", "threat_vz_mps",
]


def row_to_csv(row: dict) -> str:
    return ",".join(str(row[field]) for field in FIELDS)


def main() -> int:
    if len(sys.argv) != 2:
        print("usage: generate_synthetic_rosbag.py OUTPUT_DIR", file=sys.stderr)
        return 2

    out_dir = Path(sys.argv[1]).resolve()
    out_dir.mkdir(parents=True, exist_ok=True)
    db_path = out_dir / "autonomymind_decision_0.db3"
    if db_path.exists():
        db_path.unlink()

    conn = sqlite3.connect(db_path)
    cur = conn.cursor()
    cur.execute(
        "CREATE TABLE topics("
        "id INTEGER PRIMARY KEY,"
        "name TEXT NOT NULL,"
        "type TEXT NOT NULL,"
        "serialization_format TEXT NOT NULL,"
        "offered_qos_profiles TEXT NOT NULL)"
    )
    cur.execute(
        "CREATE TABLE messages("
        "id INTEGER PRIMARY KEY,"
        "topic_id INTEGER NOT NULL,"
        "timestamp INTEGER NOT NULL,"
        "data BLOB NOT NULL)"
    )
    cur.execute(
        "CREATE INDEX timestamp_idx ON messages (timestamp ASC)"
    )
    cur.execute(
        "INSERT INTO topics(id,name,type,serialization_format,offered_qos_profiles)"
        " VALUES(1,?,?,?,?)",
        (TOPIC, TYPE, "cdr", ""),
    )

    rows = scenario_rows()
    for message_id, row in enumerate(rows, start=1):
        timestamp_ns = int(float(row["timestamp_s"]) * 1_000_000_000)
        cur.execute(
            "INSERT INTO messages(id,topic_id,timestamp,data) VALUES(?,?,?,?)",
            (message_id, 1, timestamp_ns, cdr_string(row_to_csv(row))),
        )

    conn.commit()
    conn.close()

    metadata = {
        "rosbag2_bagfile_information": {
            "version": 5,
            "storage_identifier": "sqlite3",
            "duration": {"nanoseconds": int(rows[-1]["timestamp_s"] * 1e9)},
            "starting_time": {"nanoseconds_since_epoch": 0},
            "message_count": len(rows),
            "topics_with_message_count": [{
                "topic_metadata": {
                    "name": TOPIC,
                    "type": TYPE,
                    "serialization_format": "cdr",
                    "offered_qos_profiles": "",
                },
                "message_count": len(rows),
            }],
            "relative_file_paths": [db_path.name],
            "files": [{
                "path": db_path.name,
                "starting_time": {"nanoseconds_since_epoch": 0},
                "duration": {"nanoseconds": int(rows[-1]["timestamp_s"] * 1e9)},
                "message_count": len(rows),
            }],
        }
    }
    # JSON is valid YAML 1.2, so rosbag2 YAML parsers can consume this file.
    (out_dir / "metadata.yaml").write_text(
        json.dumps(metadata, indent=2) + "\n",
        encoding="utf-8",
    )
    (out_dir / "scenario_fields.txt").write_text(
        ",".join(FIELDS) + "\n",
        encoding="utf-8",
    )

    print(f"[PASS] generated synthetic rosbag2 dataset: {len(rows)} messages")
    print(db_path)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
