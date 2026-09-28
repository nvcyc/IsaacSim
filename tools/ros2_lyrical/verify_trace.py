#!/usr/bin/env python3
# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

"""Validate CUDA-only probe or Isaac Sim copy counts in an Nsight SQLite export."""

import argparse
import json
import sqlite3
from pathlib import Path


def verify_trace(path: Path, frames: int, runtime_direction: str | None = None) -> dict:
    """Check that D2H contains only validation scalars and D2D matches publication."""
    with sqlite3.connect(f"file:{path.resolve()}?mode=ro", uri=True) as database:
        records = database.execute(
            "SELECT copyKind, COUNT(*), SUM(bytes), MIN(bytes), MAX(bytes) "
            "FROM CUPTI_ACTIVITY_KIND_MEMCPY GROUP BY copyKind"
        ).fetchall()
        apis = dict(
            database.execute(
                "SELECT s.value, COUNT(*) FROM CUPTI_ACTIVITY_KIND_RUNTIME r "
                "JOIN StringIds s ON r.nameId=s.id GROUP BY s.value"
            ).fetchall()
        )
    # CUPTI_ACTIVITY_MEMCPY_KIND_DTOH=2, DTOD=8. Reject every other copy kind.
    rows = {kind: (count, total, minimum, maximum) for kind, count, total, minimum, maximum in records}
    expected_payload = ((frames + 1) // 2) * 640 * 480 + (frames // 2) * 1280 * 720
    expected = {
        2: (frames * 2, frames * 2 * 4, 4, 4),
        8: (frames, expected_payload, 640 * 480, 1280 * 720),
    }
    if runtime_direction:
        expected[2] = (frames, frames * 4, 4, 4)
        if runtime_direction == "subscribe":
            del expected[8]
    if rows != expected:
        raise RuntimeError(f"Unexpected payload copies: observed={rows}, expected={expected}")
    imports = sum(count for name, count in apis.items() if name.startswith("cuMemImportFromShareableHandle"))
    device_syncs = sum(
        count for name, count in apis.items() if name.startswith(("cudaDeviceSynchronize", "cuCtxSynchronize"))
    )
    if not imports or device_syncs:
        raise RuntimeError(f"Expected IPC imports and no device-wide synchronization: {imports=}, {device_syncs=}")
    return {
        "frames_per_direction": frames,
        "runtime_direction": runtime_direction,
        "copies": rows,
        "full_payload_host_copies": 0,
        "ipc_imports": imports,
        "device_synchronizations": device_syncs,
    }


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("sqlite", type=Path)
    parser.add_argument("--frames", type=int, default=30)
    parser.add_argument("--runtime-direction", choices=("publish", "subscribe"))
    arguments = parser.parse_args()
    if arguments.frames < 2:
        parser.error("At least two frames are required to exercise both image sizes")
    print(json.dumps(verify_trace(arguments.sqlite, arguments.frames, arguments.runtime_direction), indent=2))
