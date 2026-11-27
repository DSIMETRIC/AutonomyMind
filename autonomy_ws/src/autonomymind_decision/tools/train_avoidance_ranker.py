#!/usr/bin/env python3
"""Train AutonomyMind's transparent avoidance ranker from labelled CSV data.

The learned model never becomes a flight-safety authority. Runtime code applies
it only after trajectory feasibility and every Full-RTA gate have passed.
"""

from __future__ import annotations

import csv
import hashlib
import json
import math
import sys
from pathlib import Path

FEATURES = [
    "separation",
    "terrain",
    "speed_margin",
    "vertical_margin",
    "accel_margin",
    "jerk_margin",
    "yaw_margin",
    "bank_margin",
]
MANEUVERS = ["LEFT", "RIGHT", "CLIMB"]


def solve_linear_system(matrix: list[list[float]], rhs: list[float]) -> list[float]:
    """Solve Ax=b by deterministic Gaussian elimination with pivoting."""
    n = len(rhs)
    aug = [matrix[i][:] + [rhs[i]] for i in range(n)]
    for col in range(n):
        pivot = max(range(col, n), key=lambda row: abs(aug[row][col]))
        if abs(aug[pivot][col]) < 1e-12:
            raise ValueError("training matrix is singular")
        aug[col], aug[pivot] = aug[pivot], aug[col]
        scale = aug[col][col]
        aug[col] = [value / scale for value in aug[col]]
        for row in range(n):
            if row == col:
                continue
            factor = aug[row][col]
            if factor == 0.0:
                continue
            aug[row] = [
                aug[row][j] - factor * aug[col][j]
                for j in range(n + 1)
            ]
    return [aug[i][n] for i in range(n)]


def train(rows: list[dict[str, str]], ridge: float = 1e-6) -> list[float]:
    """Fit linear scores plus LEFT/RIGHT/CLIMB bias terms."""
    width = len(FEATURES) + len(MANEUVERS)
    xtx = [[0.0] * width for _ in range(width)]
    xty = [0.0] * width

    for row in rows:
        maneuver = row["maneuver"].strip().upper()
        if maneuver not in MANEUVERS:
            raise ValueError(f"invalid maneuver: {maneuver}")

        vector = [float(row[name]) for name in FEATURES]
        vector.extend(1.0 if maneuver == name else 0.0 for name in MANEUVERS)
        target = float(row["target_score"])

        if not all(math.isfinite(value) for value in vector + [target]):
            raise ValueError("non-finite training value")

        for i in range(width):
            xty[i] += vector[i] * target
            for j in range(width):
                xtx[i][j] += vector[i] * vector[j]

    for i in range(width):
        xtx[i][i] += ridge

    return solve_linear_system(xtx, xty)


def main() -> int:
    if len(sys.argv) != 4:
        print(
            "usage: train_avoidance_ranker.py TRAIN.csv MODEL_OUT MANIFEST_OUT",
            file=sys.stderr,
        )
        return 2

    dataset_path = Path(sys.argv[1])
    model_path = Path(sys.argv[2])
    manifest_path = Path(sys.argv[3])

    dataset_bytes = dataset_path.read_bytes()
    with dataset_path.open(newline="", encoding="utf-8") as stream:
        rows = list(csv.DictReader(stream))

    if len(rows) < len(FEATURES) + len(MANEUVERS):
        raise ValueError("insufficient training samples")

    coefficients = train(rows)
    weights = coefficients[: len(FEATURES)]
    biases = coefficients[len(FEATURES) :]

    model_lines = [
        "model.name=autonomymind_avoidance_ranker",
        "model.version=trained-v1",
        "feature.schema=rta-safe-v1",
    ]
    model_lines.extend(
        f"weight.{name}={value:.17g}"
        for name, value in zip(FEATURES, weights)
    )
    model_lines.extend(
        f"bias.{name.lower()}={value:.17g}"
        for name, value in zip(MANEUVERS, biases)
    )
    model_text = "\n".join(model_lines) + "\n"

    model_path.parent.mkdir(parents=True, exist_ok=True)
    model_path.write_text(model_text, encoding="utf-8")

    manifest = {
        "trainer": "autonomymind-linear-ranker-v1",
        "feature_schema": "rta-safe-v1",
        "sample_count": len(rows),
        "dataset_sha256": hashlib.sha256(dataset_bytes).hexdigest(),
        "model_file_sha256": hashlib.sha256(model_text.encode()).hexdigest(),
        "features": FEATURES,
        "maneuvers": MANEUVERS,
    }
    manifest_path.write_text(
        json.dumps(manifest, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )

    print(
        f"[PASS] trained auditable ML ranker from {len(rows)} samples "
        f"dataset_sha256={manifest['dataset_sha256']}"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
