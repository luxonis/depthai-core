#!/usr/bin/env python3
"""
``Depth`` output rate vs camera configuration.

**RVC2 with a ToF sensor (``AUTO`` → ToF):** ``depthNode.build(fps=15)`` runs the ToF backend at
15 FPS. Pre-built stereo ``Camera`` nodes are not used for depth (only the ToF sensor). Optional
``--user-preview`` can still show a left stereo stream at 30 FPS alongside ToF depth.

**Other devices / RVC2 without ToF (``AUTO`` → stereo):** left/right ``Camera`` nodes at
1280x800@30, ``Depth`` reuses them but requests stereo frames at 15 FPS via ``build(fps=15)``.
On RVC2, avoid ``--user-preview`` when using stereo depth — a second 30 FPS ``requestOutput`` on
the same ``Camera`` as the 15 FPS stereo path can stall depth after a few seconds.

The main loop uses ``tryGet`` on depth so a stalled stream does not block the UI thread.
"""

from __future__ import annotations

import argparse
import sys
import threading
import time
from pathlib import Path
from typing import Optional

import cv2
import depthai as dai

sys.path.insert(0, str(Path(__file__).resolve().parent))
import depth_example_common as dec

USER_FPS = 30.0
DEPTH_FPS = 15.0
USER_RESOLUTION = (1280, 800)


class FpsCounter:
    def __init__(self) -> None:
        self._times: list[float] = []

    def tick(self) -> None:
        self._times.append(time.monotonic())
        self._times = self._times[-100:]

    def fps(self) -> float:
        if len(self._times) <= 1:
            return 0.0
        return (len(self._times) - 1) / (self._times[-1] - self._times[0])


def depth_backend_label(device: dai.Device) -> str:
    if device.getPlatform() == dai.Platform.RVC2 and dec.device_has_tof_sensor(device):
        return "ToF"
    if device.getPlatform() == dai.Platform.RVC4:
        return "NeuralDepth"
    return "StereoDepth"


parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
parser.add_argument(
    "--user-preview",
    action="store_true",
    help="Show a 30 FPS left stereo preview (on RVC2 stereo-depth boards, can stall depth).",
)
args = parser.parse_args()

pipeline = dai.Pipeline()
try:
    device = dec.require_default_device(pipeline)
    stereo_pair = dec.require_first_stereo_pair(device)
except RuntimeError as ex:
    print(ex, file=sys.stderr)
    sys.exit(1)

platform = device.getPlatform()
rvc2_tof = platform == dai.Platform.RVC2 and dec.device_has_tof_sensor(device)
stereo_fps_demo = not rvc2_tof
backend_label = depth_backend_label(device)

enable_user_preview = args.user_preview or platform != dai.Platform.RVC2
if enable_user_preview and stereo_fps_demo and platform == dai.Platform.RVC2:
    print(
        "RVC2 + --user-preview + stereo depth: left preview may keep running while depth freezes "
        "(30 vs 15 FPS on the same Camera).",
        flush=True,
    )
elif rvc2_tof:
    print(
        f"RVC2 + ToF: Depth AUTO → {backend_label} at {DEPTH_FPS:g} FPS "
        "(stereo cameras are not used for depth).",
        flush=True,
    )
elif platform == dai.Platform.RVC2:
    print(
        f"RVC2: Depth AUTO → {backend_label}, stereo cameras at {USER_FPS:g} FPS sensor rate, "
        f"depth stereo outputs at {DEPTH_FPS:g} FPS.",
        flush=True,
    )

left_cam = None
user_queue: Optional[dai.MessageQueue] = None

if enable_user_preview or stereo_fps_demo:
    left_cam = pipeline.create(dai.node.Camera).build(
        stereo_pair.left, sensorResolution=USER_RESOLUTION, sensorFps=USER_FPS
    )
    if stereo_fps_demo:
        pipeline.create(dai.node.Camera).build(
            stereo_pair.right, sensorResolution=USER_RESOLUTION, sensorFps=USER_FPS
        )

if enable_user_preview:
    assert left_cam is not None
    user_out = left_cam.requestOutput(USER_RESOLUTION, fps=USER_FPS)
    if user_out is None:
        print("Failed to request left camera preview output.", file=sys.stderr)
        sys.exit(1)
    user_queue = user_out.createOutputQueue(maxSize=30, blocking=False)

depth_node = pipeline.create(dai.node.Depth).build(DEPTH_FPS)
depth_queue = depth_node.depth.createOutputQueue(maxSize=8, blocking=False)

pipeline.build()

user_lock = threading.Lock()
latest_user_frame: Optional[dai.ImgFrame] = None
user_fps = FpsCounter()
depth_fps = FpsCounter()
stop_event = threading.Event()


def user_drain_loop() -> None:
    global latest_user_frame
    assert user_queue is not None
    while not stop_event.is_set():
        frame = user_queue.tryGet()
        if frame is None:
            time.sleep(0.001)
            continue
        assert isinstance(frame, dai.ImgFrame)
        user_fps.tick()
        with user_lock:
            latest_user_frame = frame


drain_thread: Optional[threading.Thread] = None
if enable_user_preview:
    drain_thread = threading.Thread(target=user_drain_loop, name="user-drain", daemon=True)
    drain_thread.start()

depth_caption = f"depth {backend_label}"

with pipeline:
    pipeline.start()
    last_status = time.monotonic()
    while pipeline.isRunning():
        depth_frame = depth_queue.tryGet()
        if depth_frame is not None:
            assert isinstance(depth_frame, dai.ImgFrame)
            depth_fps.tick()
            vis = dec.colorizeDepthMm(depth_frame.getFrame())
            cv2.putText(
                vis,
                f"{depth_caption} {depth_fps.fps():.1f} fps (build {DEPTH_FPS:g})",
                (8, 24),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.6,
                (255, 255, 255),
                2,
            )
            cv2.imshow("depth", vis)

        if enable_user_preview:
            with user_lock:
                user_frame = latest_user_frame
            if user_frame is not None:
                user_vis = user_frame.getCvFrame()
                cv2.putText(
                    user_vis,
                    f"user left {user_fps.fps():.1f} fps (sensor {USER_FPS:g})",
                    (8, 24),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.6,
                    (0, 255, 0),
                    2,
                )
                cv2.imshow("user left", user_vis)

        now = time.monotonic()
        if now - last_status >= 2.0:
            if enable_user_preview:
                print(
                    f"receive rates: user left ~{user_fps.fps():.1f} fps, "
                    f"{depth_caption} ~{depth_fps.fps():.1f} fps",
                    flush=True,
                )
            else:
                print(f"receive rate: {depth_caption} ~{depth_fps.fps():.1f} fps", flush=True)
            last_status = now

        if cv2.waitKey(1) == ord("q"):
            pipeline.stop()
            break

    stop_event.set()
    if drain_thread is not None:
        drain_thread.join(timeout=1.0)
    cv2.destroyAllWindows()
