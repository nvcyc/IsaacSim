#!/usr/bin/env python3
# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

"""Validate CUDA-only two-direction probe copy counts in an Nsight SQLite export."""

import argparse
import json
import sqlite3
from pathlib import Path


def verify_trace(path: Path, frames: int) -> dict:
    """Check that D2H contains only validation scalars and D2D matches publication."""
    with sqlite3.connect(f"file:{path.resolve()}?mode=ro", uri=True) as database:
        records = database.execute(
            "SELECT copyKind, COUNT(*), SUM(bytes), MIN(bytes), MAX(bytes) "
            "FROM CUPTI_ACTIVITY_KIND_MEMCPY GROUP BY copyKind"
        ).fetchall()
    # CUPTI_ACTIVITY_MEMCPY_KIND_DTOH=2, DTOD=8. Reject every other copy kind.
    rows = {kind: (count, total, minimum, maximum) for kind, count, total, minimum, maximum in records}
    expected_payload = ((frames + 1) // 2) * 640 * 480 + (frames // 2) * 1280 * 720
    expected = {
        2: (frames * 2, frames * 2 * 4, 4, 4),
        8: (frames, expected_payload, 640 * 480, 1280 * 720),
    }
    if rows != expected:
        raise RuntimeError(f"Unexpected payload copies: observed={rows}, expected={expected}")
    return {"frames_per_direction": frames, "copies": rows, "full_payload_host_copies": 0}


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("sqlite", type=Path)
    parser.add_argument("--frames", type=int, default=30)
    arguments = parser.parse_args()
    if arguments.frames < 2:
        parser.error("At least two frames are required to exercise both image sizes")
    print(json.dumps(verify_trace(arguments.sqlite, arguments.frames), indent=2))
