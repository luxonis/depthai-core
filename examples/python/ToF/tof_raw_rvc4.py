#!/usr/bin/env python3
"""Test script: capture raw depth via Camera + ToFBase pipeline, save to temp dir, verify,
then replay the saved raw sensor frames back into a fresh ToFBase node and plot the result.

Capture pipeline:
  ┌─────────────────────────┐
  │  Camera (CAM_D, ToF)    │
  └──────────┬──────────────┘
             │ .raw
             ▼
  ┌──────────────────────────┐
  │         ToFBase          │
  └────┬──────────┬──────────┘
       │ .raw     │ .depth     (also .amplitude, not captured here)
       ▼          ▼
    raw_q      depth_q
       │          │
       ▼          ▼
   raw_<ts>.npz  depth_<ts>.npy   <-- saved and validated

Replay pipeline (no Camera node, raw frames fed from disk):
  raw_<ts>.npz --> rawInput queue --> ToFBase --> .depth --> depth_q --> matplotlib

Usage:
    python tof_raw_depth_test.py
    python tof_raw_depth_test.py --socket CAM_D --frames 5
    python tof_raw_depth_test.py --ip 192.168.1.100
"""

import argparse
import os
import sys
import tempfile
import time

import matplotlib.pyplot as plt
import numpy as np

import depthai as dai


WARMUP_FRAMES = 10


def parse_args():
    parser = argparse.ArgumentParser(description="ToF raw depth save + verify test")
    parser.add_argument("--ip", default=None, help="Device IP address (omit for USB)")
    parser.add_argument("--socket", default="CAM_D", help="ToF camera board socket (default: CAM_D)")
    parser.add_argument("--frames", type=int, default=3,
                        help="Number of depth frames to save (default: 3)")
    parser.add_argument("--fwp", default=None,
                        help="Optional path to RVC4 firmware package (.tar.xz)")
    parser.add_argument("--plot-out", default=None,
                        help="Path to save the replay depth plot PNG (default: <tempdir>/replay_depth.png)")
    return parser.parse_args()


def build_capture_pipeline(socket: dai.CameraBoardSocket, profile):
    pipeline = dai.Pipeline()

    tof_base = pipeline.create(dai.node.ToFBase)
    tof_base.build(boardSocket=socket, profile=profile)

    cam = pipeline.create(dai.node.Camera)
    cam.setSensorType(dai.CameraSensorType.TOF)
    cam.build(boardSocket=tof_base.getBoardSocket())

    cam.raw.link(tof_base.rawInput)

    # Save the frames actually fed into rawInput, not ToFBase's own (unreliable on RVC4) .raw passthrough.
    raw_q   = cam.raw.createOutputQueue()
    depth_q = tof_base.depth.createOutputQueue()

    return pipeline, raw_q, depth_q


def build_replay_pipeline(socket: dai.CameraBoardSocket, profile):
    """Pipeline with only ToFBase (no Camera) -- raw frames are pushed in from the host."""
    pipeline = dai.Pipeline()

    tof_base = pipeline.create(dai.node.ToFBase)
    tof_base.build(boardSocket=socket, profile=profile)

    raw_in_q = tof_base.rawInput.createInputQueue()
    depth_q  = tof_base.depth.createOutputQueue()

    return pipeline, raw_in_q, depth_q


def verify_saved_files(out_dir: str, expected_count: int) -> bool:
    depth_files = sorted(f for f in os.listdir(out_dir) if f.startswith("depth_") and f.endswith(".npy"))

    print(f"\n[Verify] Expected {expected_count} file(s), found {len(depth_files)}")
    if len(depth_files) < expected_count:
        print(f"[FAIL] Not enough depth files saved.")
        return False

    all_ok = True
    for fname in depth_files:
        path = os.path.join(out_dir, fname)
        size = os.path.getsize(path)
        arr = np.load(path)
        nonzero = int(np.count_nonzero(arr))
        status = "OK" if nonzero > 0 else "EMPTY"
        print(f"  {fname}: shape={arr.shape} dtype={arr.dtype} nonzero={nonzero} size={size}B [{status}]")
        if nonzero == 0:
            print(f"  [FAIL] {fname} contains only zeros.")
            all_ok = False

    return all_ok


def load_raw_frames(out_dir: str):
    """Load raw_<ts>.npz files saved during capture, sorted by timestamp."""
    raw_files = sorted(f for f in os.listdir(out_dir) if f.startswith("raw_") and f.endswith(".npz"))
    frames = []
    for fname in raw_files:
        with np.load(os.path.join(out_dir, fname)) as npz:
            data = npz["data"]
            frame_type = getattr(dai.ImgFrame.Type, str(npz["type"]))
        ts = int(fname[len("raw_"):-len(".npz")])
        frames.append((ts, data, frame_type))
    return frames


def replay_raw_frames(socket: dai.CameraBoardSocket, profile, out_dir: str):
    """Feed saved raw frames back into a fresh ToFBase node and collect the resulting depth frames."""
    raw_frames = load_raw_frames(out_dir)
    if not raw_frames:
        print("[Replay] No raw frames found to replay.")
        return []

    print(f"\n[Replay] Feeding {len(raw_frames)} saved raw frame(s) back into ToFBase...")
    pipeline, raw_in_q, depth_q = build_replay_pipeline(socket, profile)

    depth_frames = []
    with pipeline as p:
        p.start()
        for ts, data, frame_type in raw_frames:
            img = dai.ImgFrame()
            img.setCvFrame(data, frame_type)
            raw_in_q.send(img)

            depth_msg = depth_q.get()
            depth_frames.append((ts, depth_msg.getFrame()))
            print(f"[Replay] ts={ts}ms -> depth={depth_frames[-1][1].shape}")

    return depth_frames


def plot_depth_frames(depth_frames, out_path: str):
    """Plot replayed depth frames side by side and save to a PNG file."""
    n = len(depth_frames)
    fig, axes = plt.subplots(1, n, figsize=(5 * n, 4), squeeze=False)

    for ax, (ts, depth) in zip(axes[0], depth_frames):
        im = ax.imshow(depth, cmap="turbo")
        ax.set_title(f"ts={ts}ms")
        ax.axis("off")
        fig.colorbar(im, ax=ax, fraction=0.046, pad=0.04)

    fig.suptitle("Replayed ToF depth (from saved raw frames)")
    fig.tight_layout()
    fig.savefig(out_path, dpi=150)
    print(f"[Plot]   Saved to: {out_path}")

    try:
        plt.show()
    except Exception:
        pass


def main():
    args = parse_args()

    if args.ip:
        os.environ["DEPTHAI_DEVICE_NAME_LIST"] = args.ip
    if args.fwp:
        os.environ["DEPTHAI_DEVICE_RVC4_FWP"] = args.fwp

    socket = getattr(dai.CameraBoardSocket, args.socket)
    preset = dai.ToFConfig.Profile.HIGH_RANGE

    print(f"[Config] socket={args.socket}, frames={args.frames}")
    print(f"[Config] DepthAI {dai.__version__}")

    out_dir = tempfile.mkdtemp(prefix="tof_depth_test_")
    print(f"[Temp]   Saving to: {out_dir}")

    pipeline, raw_q, depth_q = build_capture_pipeline(socket, preset)

    saved = 0
    frame_count = 0
    warmup_done = False

    with pipeline as p:
        p.start()
        print(f"[Pipeline] Running (warmup={WARMUP_FRAMES} frames)...")

        t_start = time.monotonic()

        while p.isRunning() and saved < args.frames:
            depth_msg = depth_q.get()
            raw_msg   = raw_q.get()
            frame_count += 1

            if not warmup_done:
                if frame_count >= WARMUP_FRAMES:
                    warmup_done = True
                    print(f"[Warmup]  Done after {frame_count} frames. Starting capture...")
                continue

            depth_data = depth_msg.getFrame()
            ts = int(depth_msg.getTimestamp().total_seconds() * 1000)
            path = os.path.join(out_dir, f"depth_{ts}.npy")
            np.save(path, depth_data)

            raw_data = raw_msg.getFrame()
            raw_path = os.path.join(out_dir, f"raw_{ts}.npz")
            np.savez(raw_path, data=raw_data, type=np.array(raw_msg.getType().name))

            saved += 1
            print(f"[Save {saved}/{args.frames}] depth={depth_data.shape} raw={raw_data.shape} ts={ts}ms")

        elapsed = time.monotonic() - t_start
        print(f"[Done]   {frame_count} total frames in {elapsed:.1f}s")

    passed = verify_saved_files(out_dir, args.frames)

    print("\n" + ("=" * 40))
    if passed:
        print("RESULT: PASS — depth files saved and contain valid data")
    else:
        print("RESULT: FAIL — see errors above")
        sys.exit(1)

    depth_frames = replay_raw_frames(socket, preset, out_dir)
    if depth_frames:
        plot_out = args.plot_out or os.path.join(out_dir, "replay_depth.png")
        plot_depth_frames(depth_frames, plot_out)


if __name__ == "__main__":
    main()
