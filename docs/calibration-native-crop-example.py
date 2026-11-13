"""Plot ideal projections using recorded IMX214 frame intrinsics.

Run: python -m pip install numpy matplotlib
     python docs/calibration-native-crop-example.py

No device is opened. The landmarks are synthetic; the matrices are from hardware.
This isolates coordinate mapping, not calibration-target accuracy or distortion.
"""

import json
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np


def main():
    root = Path(__file__).resolve().parent
    data = json.loads((root / "calibration-native-crop-example.json").read_text())
    full = np.array(data["full_K"])
    before = np.array(data["before_K"])
    after = np.array(data["after_K"])
    x, y, width, height = data["readout_xywh"]
    crop = np.array([[1, 0, -x], [0, 1, -y], [0, 0, 1]])
    expected = crop @ full
    np.testing.assert_allclose(after, expected, rtol=0, atol=0.01)

    # Independent pixel reference: project into the full image, then subtract
    # the known physical crop origin. Do not derive targets from the tested K.
    rays = np.array([[u, v, 1] for v in [-0.23, 0, 0.23]
                     for u in [-0.5, -0.3, 0, 0.3, 0.5]]).T
    target = (full @ rays)[:2] - np.array([[x], [y]])
    old = (before @ rays)[:2]
    new = (after @ rays)[:2]
    old_error = np.linalg.norm(old - target, axis=0)
    new_error = np.linalg.norm(new - target, axis=0)

    fig, axes = plt.subplots(1, 2, figsize=(13, 5), layout="constrained")
    for ax, projected, label, color, errors in [
        (axes[0], old, "Before: native-4K metadata", "#c43832", old_error),
        (axes[1], new, "After: corrected metadata", "#087e6c", new_error),
    ]:
        ax.set_facecolor("#f5f7fa")
        ax.scatter(*target, s=120, facecolors="none", edgecolors="#24374a",
                   linewidths=1.6, label="Full-image projection minus crop origin")
        ax.scatter(*projected, marker="+", s=90, linewidths=1.8,
                   color=color, label="Projection using frame intrinsics")
        for p, q in zip(projected.T, target.T):
            ax.plot([p[0], q[0]], [p[1], q[1]], color=color, alpha=0.65)
        ax.set(xlim=(0, width), ylim=(height, 0), aspect="equal",
               xlabel="u (native 3840-pixel image)", ylabel="v (pixels)",
               title=f"{label}\nMaximum shown displacement: {errors.max():.2f} px")
        ax.grid(alpha=0.15)
        ax.legend(loc="upper center", fontsize=7)
    fig.suptitle("Same crop geometry; different intrinsics metadata\n"
                 "Synthetic ideal rays projected with recorded IMX214 matrices",
                 fontsize=14)
    output = root / "assets" / "camera-native-crop-before-after.png"
    output.parent.mkdir(exist_ok=True)
    fig.savefig(output, dpi=150)
    plt.close(fig)

    print("fx error (%):", 100 * (before[0, 0] / expected[0, 0] - 1))
    print("X/Z | before u,v | expected/after u,v | before error (px)")
    for ratio in [0, 0.1, 0.3, 0.5]:
        ray = np.array([ratio, 0, 1])
        p = (before @ ray)[:2]
        q = (full @ ray)[:2] - [x, y]
        print(f"{ratio:.1f} | {p.round(3)} | {q.round(3)} | {np.linalg.norm(p-q):.3f}")
    # A same-camera round trip can hide a wrong K by cancellation.
    pixels = np.vstack((target, np.ones(target.shape[1])))
    round_trip = before @ np.linalg.solve(before, pixels)
    print("Wrong-K round-trip error (px):", np.abs(round_trip - pixels).max())
    print("Corrected projection maximum error (px):", new_error.max())
    print("Saved", output.name)


if __name__ == "__main__":
    main()
