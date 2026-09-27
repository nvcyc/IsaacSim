#!/usr/bin/env python3
# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

"""Exercise generated C++ images through rcl in separate processes, without rclcpp."""

import argparse
import os
import subprocess
import tempfile
from contextlib import ExitStack


def run_case(
    executable: str,
    publisher: str,
    subscribers: list[tuple[str, str]],
    index: int,
    publisher_executable: str | None = None,
) -> None:
    """Run one publisher with its subscribers and reject transport or payload failures."""
    environment = dict(os.environ, RMW_IMPLEMENTATION="rmw_fastrtps_cpp", ROS_DOMAIN_ID=str(110 + index))
    topic = f"/isaac_image_probe_{index}"
    frames = os.environ.get("ISAAC_IMAGE_PROBE_FRAMES", "30")
    processes = []
    files = []
    resources = ExitStack()
    try:
        for storage, expected in subscribers:
            output_file = resources.enter_context(tempfile.TemporaryFile(mode="w+"))
            files.append(output_file)
            processes.append(
                subprocess.Popen(
                    [executable, "sub", storage, topic, frames, expected],
                    env=environment,
                    stdout=output_file,
                    stderr=subprocess.STDOUT,
                    text=True,
                )
            )
        output_file = resources.enter_context(tempfile.TemporaryFile(mode="w+"))
        files.append(output_file)
        processes.append(
            subprocess.Popen(
                [publisher_executable or executable, "pub", publisher, topic, frames, str(len(subscribers))],
                env=environment,
                stdout=output_file,
                stderr=subprocess.STDOUT,
                text=True,
            )
        )
        for process, output_file in zip(processes, files):
            process.wait(timeout=max(65, int(frames) * 0.05 + 30))
            output_file.seek(0)
            output = output_file.read()
            if process.returncode:
                print(output, end="", flush=True)
                raise RuntimeError(f"Process failed with status {process.returncode}: {process.args}")
            print(
                "\n".join(line for line in output.splitlines() if line.startswith(("received=", "published="))),
                flush=True,
            )
    finally:
        for process in processes:
            if process.poll() is None:
                process.terminate()
        for process in processes:
            try:
                process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                process.kill()
                process.wait()
        resources.close()


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("external", help="Standalone external image_probe executable")
    parser.add_argument("bridge", nargs="?", help="Optional image_bridge_probe executable")
    parser.add_argument("--case", type=int, choices=range(5), help="Run one matrix case, e.g. 2 for CUDA only")
    arguments = parser.parse_args()
    for index, case in enumerate(
        [
            ("cpu", [("cpu", "cpu")]),
            ("cpu", [("cuda", "cpu")]),
            ("cuda", [("cuda", "cuda")]),
            ("cuda", [("cpu", "cpu")]),
            ("cuda", [("cuda", "cuda"), ("cpu", "cpu")]),
        ]
    ):
        if arguments.case is not None and index != arguments.case:
            continue
        print(f"CASE {index}: {case}", flush=True)
        if arguments.bridge:
            run_case(arguments.bridge, *case, index, publisher_executable=arguments.external)
            run_case(arguments.external, *case, index, publisher_executable=arguments.bridge)
        else:
            run_case(arguments.external, *case, index)
