#!/usr/bin/env python3
"""Generate the raw frames and H.264 access units used by the record-node catalog."""

import subprocess
import sys
from pathlib import Path

import cv2
import numpy as np


def run(cmd):
    subprocess.run(cmd, check=True)


def main():
    if len(sys.argv) != 2:
        raise SystemExit("usage: make_record_test_data.py <outdir>")
    out = Path(sys.argv[1])
    out.mkdir(parents=True, exist_ok=True)

    width, height, frames = 64, 48, 20
    yuv = bytearray()
    for n in range(frames):
        y = 16 + 7 * n
        yuv += bytes([y]) * (width * height)
        yuv += bytes([128]) * (width * height // 2)
    raw = out / "plain.yuv"
    raw.write_bytes(yuv)

    mp4 = out / "plain_h264_small.mp4"
    run(
        [
            "ffmpeg",
            "-y",
            "-f",
            "rawvideo",
            "-pix_fmt",
            "yuv420p",
            "-s",
            "64x48",
            "-r",
            "30",
            "-i",
            str(raw),
            "-c:v",
            "libx264",
            "-g",
            "1",
            "-pix_fmt",
            "yuv420p",
            "-crf",
            "10",
            str(mp4),
        ]
    )
    run(
        [
            "ffmpeg",
            "-y",
            "-i",
            str(mp4),
            "-c",
            "copy",
            "-bsf:v",
            "h264_mp4toannexb",
            str(out / "plain_h264_small.h264"),
        ]
    )

    gray = np.full((48, 64, 3), 98, dtype=np.uint8)
    cv2.imwrite(str(out / "gray_64x48.jpg"), gray, [int(cv2.IMWRITE_JPEG_QUALITY), 95])
    (out / ".done").write_text("ok\n")


if __name__ == "__main__":
    main()
