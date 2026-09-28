#!/usr/bin/env python3
# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

"""Verify image transport and stop/restart in a real Isaac Sim 6.1 process.

Run with Isaac Sim's python.sh in a sourced Lyrical environment. The peer must
be the separately built image_probe executable, and the pattern library must
be libruntime_image_pattern.so from the same probe build.
"""

from __future__ import annotations

import argparse
import ctypes
import json
import os
import subprocess
import sys
import time
import traceback
from pathlib import Path


class CudaPattern:
    """Own test allocations and validate leased pixels on their consumer stream."""

    def __init__(self, library: Path) -> None:
        self.library = ctypes.CDLL(str(library.resolve()))
        pointer = ctypes.c_void_p
        signatures = {
            "cudaMalloc": [ctypes.POINTER(pointer), ctypes.c_size_t],
            "cudaFree": [pointer],
            "cudaStreamCreateWithFlags": [ctypes.POINTER(pointer), ctypes.c_uint],
            "cudaStreamSynchronize": [pointer],
            "cudaStreamDestroy": [pointer],
            "cudaMemcpyAsync": [pointer, pointer, ctypes.c_size_t, ctypes.c_int, pointer],
            "cudaProfilerStart": [],
            "cudaProfilerStop": [],
            "imageProbeFill": [pointer, ctypes.c_size_t, ctypes.c_uint32, pointer],
            "imageProbeCheck": [pointer, ctypes.c_size_t, ctypes.c_uint32, pointer, pointer],
            "imageProbeCheckIpcFd": [ctypes.c_int, ctypes.c_size_t, ctypes.c_uint32, pointer, pointer],
        }
        for name, arguments in signatures.items():
            function = getattr(self.library, name)
            function.argtypes = arguments
            function.restype = ctypes.c_int
        self.library.cudaGetErrorString.argtypes = [ctypes.c_int]
        self.library.cudaGetErrorString.restype = ctypes.c_char_p
        self.stream = pointer()
        self.image = pointer()
        self.errors = pointer()
        try:
            self.call("cudaStreamCreateWithFlags", ctypes.byref(self.stream), 1)
            self.call("cudaMalloc", ctypes.byref(self.image), 1280 * 720)
            self.call("cudaMalloc", ctypes.byref(self.errors), 4)
        except BaseException:
            self.close()
            raise

    def call(self, name: str, *arguments: object) -> None:
        """Raise on every CUDA error instead of continuing with invalid data."""
        result = getattr(self.library, name)(*arguments)
        if result:
            raise RuntimeError(f"{name}: {self.library.cudaGetErrorString(result).decode()}")

    def fill(self, size: int, sequence: int) -> int:
        """Prepare a renderer-like device allocation before graph execution."""
        self.call("imageProbeFill", self.image, size, sequence, self.stream)
        self.call("cudaStreamSynchronize", self.stream)
        return self.image.value

    def check(self, pointer: int, size: int, sequence: int, stream: int) -> None:
        """Check the full image on the GPU and copy back only four result bytes."""
        self.call("imageProbeCheck", pointer, size, sequence, self.errors, stream)
        errors = ctypes.c_uint()
        self.call("cudaMemcpyAsync", ctypes.byref(errors), self.errors, 4, 2, stream)
        self.call("cudaStreamSynchronize", stream)
        if errors.value:
            raise RuntimeError(f"Frame {sequence}: {errors.value} corrupted pixels")

    def close(self) -> None:
        """Release test allocations after all graph consumers have stopped."""
        if self.stream.value:
            self.call("cudaStreamSynchronize", self.stream)
        for pointer in (self.errors, self.image):
            if pointer.value:
                self.call("cudaFree", pointer)
                pointer.value = None
        if self.stream.value:
            self.call("cudaStreamDestroy", self.stream)
            self.stream.value = None


class NitrosObserver:
    """Check legacy descriptors and GPU pixels using a same-process FD import."""

    def __init__(self, topic: str) -> None:
        import rclpy
        from isaac_ros_nitros_bridge_interfaces.msg import NitrosBridgeImage

        self.rclpy = rclpy
        if not rclpy.ok():
            rclpy.init()
        self.node = rclpy.create_node("nitros_runtime_verifier")
        self.messages = []
        self.subscription = self.node.create_subscription(NitrosBridgeImage, topic, self.messages.append, 100)
        self.topic = topic

    def wait_for_publisher(self) -> None:
        """Wait for the optional publisher before sending its first image."""
        deadline = time.monotonic() + 15
        while self.node.count_publishers(self.topic) != 1:
            if time.monotonic() > deadline:
                raise TimeoutError("The legacy NITROS publisher was not created")
            self.rclpy.spin_once(self.node, timeout_sec=0.05)

    def verify(self, pattern: CudaPattern, sequence: int, width: int, height: int) -> None:
        """Import the exported GPU allocation and check its full payload."""
        deadline = time.monotonic() + 15
        while not self.messages:
            if time.monotonic() > deadline:
                raise TimeoutError(f"NITROS frame {sequence} did not arrive")
            self.rclpy.spin_once(self.node, timeout_sec=0.05)
        message = self.messages.pop(0)
        if (message.header.stamp.sec, message.width, message.height, message.step, message.encoding) != (
            sequence,
            width,
            height,
            width,
            "mono8",
        ):
            raise RuntimeError("Incorrect NITROS image metadata")
        if len(message.data) != 2 or message.data[0] != os.getpid():
            raise RuntimeError("Unexpected NITROS descriptor process or handle format")
        pattern.call("imageProbeCheckIpcFd", message.data[1], width * height, sequence, pattern.errors, pattern.stream)

    def close(self) -> None:
        """Destroy the test subscriber before Kit shuts down its ROS extension."""
        self.node.destroy_node()


def run(options: argparse.Namespace) -> None:
    """Run a real OmniGraph endpoint against a separate ROS process."""
    options.output.parent.mkdir(parents=True, exist_ok=True)
    options.output.unlink(missing_ok=True)
    version = (options.isaac_root / "VERSION").read_text().strip()
    if not version.startswith("6.1."):
        raise RuntimeError(f"This acceptance test requires Isaac Sim 6.1, found {version}")
    if os.environ.get("ROS_DISTRO") != options.distro:
        raise RuntimeError(f"Source the {options.distro} environment before launching Isaac Sim")

    from isaacsim import SimulationApp

    app = SimulationApp({"headless": True, "renderer": "RayTracedLighting", "fast_shutdown": not options.full_shutdown})
    pattern = None
    process = None
    nitros = None
    profiling = False
    report = None
    try:
        import carb
        import isaacsim.core.experimental.utils.app as app_utils
        import omni.graph.core as og
        import omni.kit.app
        import omni.timeline
        import omni.usd

        app_utils.enable_extension("isaacsim.ros2.bridge")
        app.update()
        omni.usd.get_context().new_stage()
        app.update()
        maps = Path("/proc/self/maps").read_text()
        if f"libisaacsim.ros2.core.{options.distro}.so" not in maps:
            raise RuntimeError(f"The running simulator has not loaded the {options.distro} backend")
        pattern = CudaPattern(options.pattern_library)
        timeline = omni.timeline.get_timeline_interface()
        graph_path = "/LyricalImageAcceptance"
        endpoint = f"{graph_path}/Image"
        topic = "/isaac_runtime_image"
        if options.nitros:
            carb.settings.get_settings().set_bool("/exts/isaacsim.ros2.bridge/enable_nitros_bridge", True)
            nitros = NitrosObserver(f"{topic}/nitros_bridge")
        receive = options.direction == "subscribe"
        node_type = "ROS2SubscribeImage" if receive else "ROS2PublishImage"
        values = [("Image.inputs:topicName", topic), ("Image.inputs:queueSize", 100)]
        if receive:
            values.append(("Image.inputs:cudaDeviceIndex", 0))
        else:
            values.extend(
                [
                    ("Image.inputs:cudaDeviceIndex", 0 if options.storage == "cuda" else -1),
                    ("Image.inputs:encoding", "mono8"),
                    ("Image.inputs:frameId", "runtime_pattern"),
                    ("Image.inputs:width", 640),
                    ("Image.inputs:height", 480),
                    ("Image.inputs:bufferSize", 640 * 480),
                ]
            )
        og.Controller.edit(
            {"graph_path": graph_path, "evaluator_name": "execution"},
            {
                og.Controller.Keys.CREATE_NODES: [
                    ("Tick", "omni.graph.action.OnImpulseEvent"),
                    ("Image", f"isaacsim.ros2.bridge.{node_type}"),
                ],
                og.Controller.Keys.CONNECT: [("Tick.outputs:execOut", "Image.inputs:execIn")],
                og.Controller.Keys.SET_VALUES: values,
            },
        )

        def pulse() -> None:
            og.Controller.set(og.Controller.attribute(f"{graph_path}/Tick.state:enableImpulse"), True)
            app.update()

        def output(name: str) -> object:
            return og.Controller.get(og.Controller.attribute(f"{endpoint}.outputs:{name}"))

        def set_input(name: str, value: object) -> None:
            og.Controller.set(og.Controller.attribute(f"{endpoint}.inputs:{name}"), value)

        results = []
        options.output.parent.mkdir(parents=True, exist_ok=True)
        for cycle in range(options.cycles):
            timeline.play()
            pulse()  # Initialize the actual graph node and its rcl endpoint.
            # Finish first-play renderer/physics setup before starting the peer.
            # A live source can otherwise recycle its first CUDA buffers while
            # the simulator is still preparing its initial frames.
            for _ in range(15):
                app.update()
            print(f"Runtime endpoint ready: cycle={cycle + 1}, direction={options.direction}", flush=True)
            if nitros is not None:
                nitros.wait_for_publisher()
            if options.profile:
                pattern.call("cudaProfilerStart")
                profiling = True
            arguments = [
                str(options.peer.resolve()),
                "pub" if receive else "sub",
                options.peer_storage,
                topic,
                str(options.frames),
            ]
            expected_backend = (
                "cuda"
                if options.distro == "lyrical" and options.storage == "cuda" and options.peer_storage == "cuda"
                else "cpu"
            )
            arguments.append("1" if receive else expected_backend)
            if not receive:
                arguments.append("stamp")
            environment = dict(os.environ)
            if options.peer_ros_prefix:
                # Keep each distro's shared libraries in its own process. Pass
                # paths as shell arguments so filenames cannot become commands.
                environment.pop("ROS_DISTRO", None)
                environment["COLCON_PYTHON_EXECUTABLE"] = sys.executable
                arguments = [
                    "/bin/bash",
                    "-c",
                    'set -e; source "$1/setup.bash"; shift; exec "$@"',
                    "image-peer",
                    str(options.peer_ros_prefix.resolve()),
                    *arguments,
                ]
            peer_log_path = options.output.with_name(f"{options.output.stem}.cycle-{cycle + 1}.peer.log")
            with peer_log_path.open("w+") as peer_log:
                process = subprocess.Popen(
                    arguments, env=environment, stdout=peer_log, stderr=subprocess.STDOUT, text=True
                )
                deadline = time.monotonic() + 90
                if receive:
                    previous = 0
                    while previous < options.frames:
                        if time.monotonic() > deadline:
                            raise TimeoutError("Simulator image reception timed out")
                        pulse()
                        pointer = output("dataPtr")
                        if not pointer:
                            if process.poll() not in (None, 0):
                                raise RuntimeError("External publisher failed")
                            time.sleep(0.001)
                            continue
                        sequence = int(output("frameId"))
                        if sequence != previous + 1:
                            raise RuntimeError(f"Missing/duplicate frame: previous={previous}, received={sequence}")
                        width, height = (640, 480) if sequence % 2 else (1280, 720)
                        if (output("width"), output("height"), output("step"), output("encoding")) != (
                            width,
                            height,
                            width,
                            "mono8",
                        ):
                            raise RuntimeError("Invalid image metadata from the OmniGraph subscriber")
                        if output("bufferSize") != width * height:
                            raise RuntimeError("Incorrect image payload length")
                        if output("promotedFromHost") != (options.peer_storage == "cpu"):
                            raise RuntimeError("Unexpected transport fallback")
                        pattern.check(pointer, width * height, sequence, output("cudaStream"))
                        previous = sequence
                    # The node retains the most recent lease until its next
                    # evaluation. Release it after validation so the external
                    # publisher can finish tearing down its CUDA allocation pool.
                    pulse()
                else:
                    # Wait for DDS matching without triggering an extra publication.
                    while True:
                        peer_log.seek(0)
                        if "ready=subscriber" in peer_log.read():
                            break
                        if process.poll() is not None or time.monotonic() > deadline:
                            raise RuntimeError("External subscriber failed to match the graph publisher")
                        app.update()
                        time.sleep(0.01)
                    host_pixels = None
                    for sequence in range(1, options.frames + 1):
                        # The legacy NITROS pool is fixed at its initial size.
                        width, height = (640, 480) if sequence % 2 or options.nitros else (1280, 720)
                        size = width * height
                        if options.storage == "cuda":
                            pointer = pattern.fill(size, sequence)
                        else:
                            import numpy as np

                            host_pixels = ((np.arange(size, dtype=np.uint32) * 13 + sequence * 37) % 256).astype(
                                np.uint8
                            )
                            pointer = host_pixels.ctypes.data
                        for name, value in (
                            ("width", width),
                            ("height", height),
                            ("bufferSize", size),
                            ("dataPtr", pointer),
                            ("timeStamp", float(sequence)),
                        ):
                            set_input(name, value)
                        pulse()
                        if nitros is not None:
                            nitros.verify(pattern, sequence, width, height)
                        time.sleep(0.033)
                result = process.wait(timeout=20)
                peer_log.seek(0)
                log = peer_log.read()
                print(log, flush=True)
                if result:
                    raise RuntimeError(f"External endpoint failed with exit code {result}")
                process = None
            timeline.stop()
            app.update()
            app.update()
            if receive and (output("dataPtr") or output("cudaStream") or output("bufferSize")):
                raise RuntimeError("Stop left a stale GPU pointer, stream, or payload size in the graph")
            results.append({"cycle": cycle + 1, "frames": options.frames, "payload": "verified", "stop": "passed"})
            if profiling:
                pattern.call("cudaProfilerStop")
                profiling = False
        omni.usd.get_context().get_stage().RemovePrim(graph_path)
        app.update()
        report = {
            "status": "passed",
            "isaac_version": version,
            "distro": options.distro,
            "direction": options.direction,
            "storage": options.storage,
            "peer_storage": options.peer_storage,
            "nitros_gpu_payload_verified": options.nitros,
            "shutdown": "pending" if options.full_shutdown else "fast",
            "cycles": results,
            "kit_version": omni.kit.app.get_app().get_build_version(),
            "app_version": carb.settings.get_settings().get("/app/version"),
        }
        options.output.write_text(json.dumps(report, indent=2) + "\n")
        print(json.dumps(report), flush=True)
    except BaseException as error:
        # Preserve the original test failure even if Kit shutdown also fails.
        traceback.print_exc()
        options.output.parent.mkdir(parents=True, exist_ok=True)
        options.output.write_text(json.dumps({"status": "failed", "error": str(error)}, indent=2) + "\n")
        raise
    finally:
        if process is not None and process.poll() is None:
            process.terminate()
            try:
                process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                process.kill()
                process.wait()
        try:
            if nitros is not None:
                nitros.close()
            if pattern is not None:
                if profiling:
                    pattern.call("cudaProfilerStop")
                pattern.close()
        finally:
            app.close(exit_code=1 if sys.exc_info()[0] is not None else 0)
            if options.full_shutdown and report is not None:
                # Static destructors still run after Python exits. A successful
                # close return cannot substitute for checking the process code.
                report["shutdown"] = "returned"
                options.output.write_text(json.dumps(report, indent=2) + "\n")
                print("Full extension shutdown returned successfully", flush=True)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--isaac-root", type=Path, required=True)
    parser.add_argument("--peer", type=Path, required=True)
    parser.add_argument("--pattern-library", type=Path, required=True)
    parser.add_argument("--direction", choices=("publish", "subscribe"), required=True)
    parser.add_argument("--distro", choices=("lyrical", "humble", "jazzy"), default="lyrical")
    parser.add_argument("--peer-ros-prefix", type=Path, help="Source the external peer's separate Lyrical installation")
    parser.add_argument("--storage", choices=("cpu", "cuda"), default="cuda")
    parser.add_argument("--peer-storage", choices=("cpu", "cuda"), default="cuda")
    parser.add_argument("--frames", type=int, default=30)
    parser.add_argument("--cycles", type=int, default=2)
    parser.add_argument("--profile", action="store_true", help="Bracket one warmed-up cycle with the CUDA profiler API")
    parser.add_argument("--full-shutdown", action="store_true", help="Disable fast exit and verify extension teardown")
    parser.add_argument("--nitros", action="store_true", help="Also validate the legacy NITROS GPU allocation locally")
    parser.add_argument("--output", type=Path, required=True)
    arguments = parser.parse_args()
    if arguments.frames < 2 or arguments.cycles < 1:
        parser.error("Use at least two frames and one cycle")
    if arguments.profile and arguments.cycles != 1:
        parser.error("Use --cycles 1 when profiling")
    if arguments.direction == "subscribe" and arguments.storage != "cuda":
        parser.error("The typed subscriber exposes GPU memory; CPU transport is selected with --peer-storage cpu")
    if arguments.distro != "lyrical" and arguments.direction != "publish":
        parser.error("The typed GPU image subscriber is specific to Lyrical")
    if arguments.nitros and (arguments.distro == "lyrical" or arguments.direction != "publish"):
        parser.error("NITROS verification applies to Humble/Jazzy publication")
    run(arguments)
