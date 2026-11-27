#!/usr/bin/env python3
"""Generate the video and image files used by the replay-node catalog."""

import argparse
import json
import subprocess
import sys
from pathlib import Path

import cv2
import numpy as np


def run(cmd):
    subprocess.run(cmd, check=True)


def yuv_bytes(width, height, count):
    blob = bytearray()
    for n in range(count):
        # 16+7n exceeds a byte after frame 34. Later frames keep the low 8 bits.
        # The 30-frame files stay inside 16..219, which the catalog checks.
        y = (16 + 7 * n) & 255
        blob += bytes([y]) * (width * height)
        blob += bytes([128]) * (width * height // 2)
    return bytes(blob)


def encode(raw, out, width, height, rate, codec_args):
    run(
        [
            "ffmpeg",
            "-y",
            "-f",
            "rawvideo",
            "-pix_fmt",
            "yuv420p",
            "-s",
            f"{width}x{height}",
            "-r",
            str(rate),
            "-i",
            str(raw),
            *codec_args,
            str(out),
        ]
    )


def mean_y(path, pix_fmt, width, height, count):
    proc = subprocess.run(
        ["ffmpeg", "-v", "error", "-i", str(path), "-f", "rawvideo", "-pix_fmt", pix_fmt, "-"],
        check=True,
        stdout=subprocess.PIPE,
    )
    frame = width * height
    chroma = width * height // 2
    step = frame + chroma
    values = []
    for n in range(count):
        plane = proc.stdout[n * step : n * step + frame]
        if len(plane) != frame:
            raise SystemExit(f"short decode for {path}")
        values.append(sum(plane) / frame)
    return values


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("outdir")
    parser.add_argument("--perf", action="store_true")
    args = parser.parse_args()
    out = Path(args.outdir)
    out.mkdir(parents=True, exist_ok=True)

    raw = out / "plain.yuv"
    raw.write_bytes(yuv_bytes(1920, 1080, 30))
    encode(raw, out / "plain_h264.mp4", 1920, 1080, 30, ["-c:v", "libx264", "-profile:v", "high", "-pix_fmt", "yuv420p", "-crf", "10", "-g", "10"])
    encode(raw, out / "plain_h265.mp4", 1920, 1080, 30, ["-c:v", "libx265", "-pix_fmt", "yuv420p", "-crf", "10", "-tag:v", "hvc1"])
    encode(raw, out / "plain_ffv1.avi", 1920, 1080, 30, ["-c:v", "ffv1", "-level", "3", "-pix_fmt", "yuv420p"])
    run(
        [
            "ffmpeg",
            "-y",
            "-f",
            "rawvideo",
            "-pix_fmt",
            "yuvj420p",
            "-s",
            "1920x1080",
            "-r",
            "30",
            "-i",
            str(raw),
            "-c:v",
            "mjpeg",
            "-q:v",
            "2",
            "-pix_fmt",
            "yuvj420p",
            str(out / "plain_mjpeg.mp4"),
        ]
    )
    encode(raw, out / "plain_vp9.mp4", 1920, 1080, 30, ["-c:v", "libvpx-vp9", "-pix_fmt", "yuv420p"])
    encode(raw, out / "plain_h265_10bit.mp4", 1920, 1080, 30, ["-c:v", "libx265", "-pix_fmt", "yuv420p10le"])

    raw60 = out / "plain60.yuv"
    raw60.write_bytes(yuv_bytes(1920, 1080, 60))
    encode(raw60, out / "plain60_h264.mp4", 1920, 1080, 30, ["-c:v", "libx264", "-profile:v", "high", "-pix_fmt", "yuv420p", "-crf", "10", "-g", "10"])
    encode(raw60, out / "plain60_ffv1.avi", 1920, 1080, 30, ["-c:v", "ffv1", "-level", "3", "-pix_fmt", "yuv420p"])

    small = out / "plain_small.yuv"
    small.write_bytes(yuv_bytes(64, 48, 20))
    encode(small, out / "plain_h264_small.mp4", 64, 48, 30, ["-c:v", "libx264", "-g", "1", "-pix_fmt", "yuv420p", "-crf", "10"])
    run(["ffmpeg", "-y", "-i", str(out / "plain_h264_small.mp4"), "-c", "copy", "-bsf:v", "h264_mp4toannexb", str(out / "plain_h264_small.h264")])

    gray = np.full((48, 64, 3), 98, dtype=np.uint8)
    cv2.imwrite(str(out / "gray100.jpg"), gray, [int(cv2.IMWRITE_JPEG_QUALITY), 95])
    cv2.imwrite(str(out / "gray100.png"), gray)
    wide = np.full((481, 641, 3), 98, dtype=np.uint8)
    cv2.imwrite(str(out / "gray100_641.jpg"), wide, [int(cv2.IMWRITE_JPEG_QUALITY), 95])
    rgba = np.full((48, 64, 4), (98, 98, 98, 255), dtype=np.uint8)
    cv2.imwrite(str(out / "rgba.png"), rgba)
    png16 = np.full((48, 64), 25000, dtype=np.uint16)
    cv2.imwrite(str(out / "png16.png"), png16)

    import zipfile

    with zipfile.ZipFile(out / "cam.zip", "w") as archive:
        archive.writestr("cam_0.dai", b"\x00" * 10)

    means = {}
    for name, pix in (
        ("plain_h264.mp4", "yuv420p"),
        ("plain_h265.mp4", "yuv420p"),
        ("plain_ffv1.avi", "yuv420p"),
        ("plain_mjpeg.mp4", "yuvj420p"),
    ):
        means[name] = mean_y(out / name, pix, 1920, 1080, 30)
    (out / "expected_meanY.json").write_text(json.dumps(means))

    if args.perf:
        perf = out / "perf.yuv"
        # 600 frames, the 30-frame pattern repeated.
        pattern = yuv_bytes(3840, 2160, 30)
        perf.write_bytes(pattern * 20)
        encode(perf, out / "perf_4k_h264.mp4", 3840, 2160, 30, ["-c:v", "libx264", "-pix_fmt", "yuv420p", "-crf", "18", "-g", "30"])
        hd = out / "perf_hd.yuv"
        hd.write_bytes(yuv_bytes(1920, 1080, 30) * 20)
        encode(hd, out / "perf_1080_ffv1.avi", 1920, 1080, 30, ["-c:v", "ffv1", "-level", "3", "-pix_fmt", "yuv420p"])
        for name, crf in (("perf_1080_h264_a.mp4", "18"), ("perf_1080_h264_b.mp4", "19"), ("perf_1080_h264_c.mp4", "20")):
            encode(hd, out / name, 1920, 1080, 30, ["-c:v", "libx264", "-pix_fmt", "yuv420p", "-crf", crf, "-g", "30"])

    (out / ".done").write_text("ok\n")


if __name__ == "__main__":
    main()
