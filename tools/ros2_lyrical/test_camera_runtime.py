#!/usr/bin/env python3
# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

"""Compare a rendered CUDA image with the image delivered by the real ROS node.

Run with Isaac Sim's python.sh after sourcing the selected ROS environment.
A GPU snapshot keeps renderer storage stable during verification. Reading that
snapshot to CPU supplies the reference; this is a correctness test, not a copy
profiling workload.
"""

import argparse
import hashlib
import json
import os
import sys
import time
import traceback
from pathlib import Path


def run(options: argparse.Namespace) -> None:
    """Render local geometry and verify every RGBA byte through the publisher."""
    options.output.parent.mkdir(parents=True, exist_ok=True)
    options.output.unlink(missing_ok=True)
    version = (options.isaac_root / "VERSION").read_text().strip()
    if not version.startswith("6.1."):
        raise RuntimeError(f"Isaac Sim 6.1 is required, found {version}")

    from isaacsim import SimulationApp

    app = SimulationApp({"headless": True, "renderer": "RayTracedLighting"})
    node = None
    try:
        import isaacsim.core.experimental.utils.app as app_utils
        import numpy as np
        import omni.graph.core as og
        import omni.replicator.core as rep
        import omni.timeline
        import omni.usd
        import warp as wp
        from PIL import Image as PillowImage
        from pxr import Gf, UsdGeom, UsdLux

        app_utils.enable_extension("isaacsim.ros2.bridge")
        app.update()
        distro = os.environ["ROS_DISTRO"]
        if f"libisaacsim.ros2.core.{distro}.so" not in Path("/proc/self/maps").read_text():
            raise RuntimeError(f"The {distro} backend is not loaded")
        import rclpy
        from sensor_msgs.msg import Image

        if not rclpy.ok():
            rclpy.init()
        omni.usd.get_context().new_stage()
        stage = omni.usd.get_context().get_stage()
        cube = UsdGeom.Cube.Define(stage, "/World/Cube")
        cube.CreateSizeAttr(1.5)
        cube.CreateDisplayColorAttr([Gf.Vec3f(0.8, 0.1, 0.05)])
        rotation = cube.AddRotateYOp()
        camera = UsdGeom.Camera.Define(stage, "/World/Camera")
        camera.AddTranslateOp().Set(Gf.Vec3d(0, 0, 5))
        camera.CreateFocalLengthAttr(35)
        UsdLux.DomeLight.Define(stage, "/World/Light").CreateIntensityAttr(1000)
        width, height = 320, 240
        render_product = rep.create.render_product(str(camera.GetPath()), (width, height))
        annotator = rep.AnnotatorRegistry.get_annotator("rgb", device="cuda")
        annotator.attach(render_product)
        graph_path = "/RenderedImageAcceptance"
        og.Controller.edit(
            {"graph_path": graph_path, "evaluator_name": "execution"},
            {
                og.Controller.Keys.CREATE_NODES: [
                    ("Tick", "omni.graph.action.OnImpulseEvent"),
                    ("Image", "isaacsim.ros2.bridge.ROS2PublishImage"),
                ],
                og.Controller.Keys.CONNECT: [("Tick.outputs:execOut", "Image.inputs:execIn")],
                og.Controller.Keys.SET_VALUES: [
                    ("Image.inputs:topicName", "/rendered_image_acceptance"),
                    ("Image.inputs:frameId", "rendered_camera"),
                    ("Image.inputs:cudaDeviceIndex", 0),
                    ("Image.inputs:encoding", "rgba8"),
                    ("Image.inputs:width", width),
                    ("Image.inputs:height", height),
                    ("Image.inputs:bufferSize", width * height * 4),
                ],
            },
        )
        messages = []
        node = rclpy.create_node("rendered_image_verifier")
        subscription = node.create_subscription(Image, "/rendered_image_acceptance", messages.append, 10)
        timeline = omni.timeline.get_timeline_interface()
        timeline.play()

        def pulse() -> None:
            og.Controller.set(og.Controller.attribute(f"{graph_path}/Tick.state:enableImpulse"), True)
            app.update()

        pulse()
        for _ in range(30):
            app.update()
        deadline = time.monotonic() + 30
        while node.count_publishers("/rendered_image_acceptance") != 1:
            if time.monotonic() > deadline:
                raise TimeoutError("Rendered image subscriber did not discover the graph publisher")
            app.update()
            rclpy.spin_once(node, timeout_sec=0.01)

        records = []
        for sequence in range(1, options.frames + 1):
            rotation.Set(sequence * 7.0)
            app.update()
            pixels = annotator.get_data()
            if not isinstance(pixels, wp.array) or pixels.device.is_cpu:
                raise RuntimeError("The renderer annotator did not provide CUDA storage")
            # Keep the source stable across the next renderer update without
            # moving its publication path through CPU memory.
            snapshot = wp.clone(pixels)
            expected = snapshot.numpy().reshape(height, width, 4)
            if np.ptp(expected[:, :, :3]) == 0:
                raise RuntimeError("The camera produced a uniform image instead of the test geometry")
            og.Controller.set(og.Controller.attribute(f"{graph_path}/Image.inputs:dataPtr"), snapshot.ptr)
            og.Controller.set(og.Controller.attribute(f"{graph_path}/Image.inputs:timeStamp"), float(sequence))
            pulse()
            deadline = time.monotonic() + 15
            while not messages:
                if time.monotonic() > deadline:
                    raise TimeoutError(f"Rendered frame {sequence} was not delivered")
                rclpy.spin_once(node, timeout_sec=0.05)
            message = messages.pop(0)
            if (message.header.stamp.sec, message.width, message.height, message.step, message.encoding) != (
                sequence,
                width,
                height,
                width * 4,
                "rgba8",
            ):
                raise RuntimeError("The rendered image metadata changed in transport")
            actual = np.frombuffer(message.data, dtype=np.uint8).reshape(height, width, 4)
            np.testing.assert_array_equal(actual, expected)
            records.append({"frame": sequence, "sha256": hashlib.sha256(actual.tobytes()).hexdigest()})
        PillowImage.fromarray(expected).save(options.output.with_suffix(".png"))
        timeline.stop()
        app.update()
        node.destroy_subscription(subscription)
        report = {
            "status": "passed",
            "isaac_version": version,
            "distro": distro,
            "source": "renderer CUDA annotator with a D2D snapshot for lifetime isolation",
            "destination": "ordinary CPU ROS Image subscriber",
            "dimensions": [width, height, 4],
            "frames": records,
            "payload": "every byte matched the rendered reference",
        }
        options.output.write_text(json.dumps(report, indent=2) + "\n")
        print(json.dumps(report), flush=True)
    except BaseException as error:
        traceback.print_exc()
        options.output.write_text(json.dumps({"status": "failed", "error": str(error)}, indent=2) + "\n")
        raise
    finally:
        if node is not None:
            node.destroy_node()
        app.close(exit_code=1 if sys.exc_info()[0] is not None else 0)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--isaac-root", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--frames", type=int, default=5)
    arguments = parser.parse_args()
    if arguments.frames < 1:
        parser.error("At least one frame is required")
    run(arguments)
