# Synthetic multi-device testing

This guide shows how to generate a **synthetic multi-device recording with ground truth** and run the whole
multi-device stack against it — with no hardware at all. It exercises the same host nodes a real rig would
(`MultiDeviceCalibration`, `CoordinateFrameTransform`, planar `Stitching`) plus the C++ unit test of the inter-device
metric-scale estimator, so it doubles as a regression suite for the calibrated bird's-eye-view path.

Everything below runs offline. The only thing that needs the real rig is a *real* recording; the synthetic one
replaces it for development.

---

## 0) What you get

`multi_device_synthetic_record.py` renders three virtual devices (a `CAM_B` + `CAM_C` stereo pair each) standing
around a textured room, looking at a floor with boxes and a calibration chart swept through the volume they share. It
writes the exact layout `multi_device_record.py` produces, plus the ground truth only a synthetic scene can provide:

| File | Contents |
| --- | --- |
| `<deviceId>_<SOCKET>.avi` / `.mcap` | One recorded stream per camera |
| `<deviceId>_calibration.json` | Factory calibration of every virtual device (intrinsics + stereo extrinsics) |
| `session.json` | Manifest of the streams and the floor plane, in the reference frame |
| `rig.json` | The **exact** inter-device poses the frames were rendered with |
| `rig_guess.json` | The same rig perturbed by `--guess-error` (8°, 25 cm), a realistic estimation seed |
| `ground_truth.json` | Every camera's pose/intrinsics/FOV in the reference frame, the plane, and the inter-device distances |

Because the geometry is known exactly, the two replay modes become **checks**: a bird's-eye view built from `rig.json`
must show the floor grid running straight through the seams, and an estimation seeded with `rig_guess.json` must come
back to `rig.json` — including the metric inter-device distances, which the estimator recovers from the scene.

---

## 1) Prerequisites

### Build depthai (Python) with dynamic calibration

The synthetic path needs the `MultiDeviceCalibration`, `CoordinateFrameTransform` and `Stitching` nodes, which live in
this branch, so build the Python module from source with dynamic-calibration support (it pulls OpenCV, which the
estimator also uses):

```bash
cmake -S . -B build -G Ninja \
    -D CMAKE_BUILD_TYPE=Release \
    -D DEPTHAI_DYNAMIC_CALIBRATION_SUPPORT=ON \
    -D DEPTHAI_BUILD_PYTHON=ON
cmake --build build --target depthai -j"$(nproc)"

# make the freshly built module importable
export PYTHONPATH="$PWD/build/bindings/python"
python3 -c "import depthai; print(depthai.__version__)"
```

### Python packages

```bash
python3 -m pip install numpy opencv-python-headless
```

`opencv-python-headless` is enough for rendering and the replay; use `opencv-python` if you want the live preview
windows (`multi_device_replay.py -m stitch` without `-o`).

---

## 2) Generate the synthetic recording

```bash
cd examples/python/MultiDevice
python3 multi_device_synthetic_record.py -o rec_synth
```

It prints the ground-truth inter-device distances as it finishes, e.g.:

```text
Wrote a synthetic recording of 6 cameras to rec_synth
  ground truth SYNTH0:CAM_B <-> SYNTH1:CAM_B: 78.3 cm
  ground truth SYNTH0:CAM_B <-> SYNTH2:CAM_B: 155.3 cm
  ground truth SYNTH1:CAM_B <-> SYNTH2:CAM_B: 78.3 cm
```

Useful knobs (all optional):

| Flag | Meaning | Default |
| --- | --- | --- |
| `-n, --frames` | Frames per stream | 24 |
| `--azimuth` | Where the devices stand around the scene, in degrees | `-15 0 15` |
| `--hfov` | Horizontal field of view of each device | `72 90 72` |
| `--radius`, `--height` | Distance from the scene centre / height above the floor, in cm | 300 / 150 |
| `--guess-error` | Rotation (deg) and translation (cm) error baked into `rig_guess.json` | `8 25` |
| `--noise`, `--seed` | Sensor-noise sigma / RNG seed for textures and noise | 2.0 / 7 |

The default `-15 0 15` spread is a **cooperative** rig — enough shared, textured overlap for the estimator to recover
metric scale. Pulling the devices apart reproduces the hard case (see step 6).

---

## 3) C++ unit test — the scale math

The estimator's math (Kabsch, RANSAC, metric triangulation) and an end-to-end rendered two-device scene are covered by
an on-host Catch2 test that needs no recording and no device:

```bash
cmake -S . -B build -G Ninja \
    -D DEPTHAI_DYNAMIC_CALIBRATION_SUPPORT=ON \
    -D DEPTHAI_BUILD_TESTS=ON
cmake --build build --target multi_device_scale_test -j"$(nproc)"
ctest --test-dir build -R multi_device_scale_test --output-on-failure
```

Expected: `100% tests passed`. The suite asserts exact rigid-transform recovery, translation-magnitude recovery under
30 % outliers, metric triangulation of a known depth, and that a rendered two-device scene recovers the inter-device
distance within tolerance.

---

## 4) Calibrate replay — recover the rig from the recording

This runs `MultiDeviceCalibration` on the recorded streams, seeded with `rig_guess.json`, and writes the estimated
rig. With the inter-device scale estimator on (the default) the node recovers the **metric** distances from the scene:

```bash
python3 multi_device_replay.py -i rec_synth -m calibrate --rig rig_guess.json --samples 16 -o rig_estimated.json
```

Expected output (numbers vary by a few mm with the seed):

```text
passed=True confidence=0.939 info='recovered the metric distance between SYNTH0 and SYNTH1 from the scene: 78.4 cm
(24795 inliers, rmse 9.6 cm); recovered the metric distance between SYNTH0 and SYNTH2 from the scene: 154.8 cm
(22907 inliers, rmse 10.8 cm); recovered the metric distance between SYNTH1 and SYNTH2 from the scene: 78.0 cm
(25266 inliers, rmse 8.5 cm)'
rig written to rig_estimated.json
```

The recovered distances match the `ground truth …` lines from step 2 to well under a centimetre.

> The estimator uses ASIFT (affine-simulated SIFT), which is CPU-heavy — expect a few minutes for 16 samples of six
> 1280×800 cameras on a typical laptop. Drop `--samples` to iterate faster.

### The scale estimator

`MultiDeviceCalibration` recovers the metric inter-device translation from the shared scene, so you do **not** have to
measure a baseline for a rig whose devices expose a stereo pair (`CAM_B` + `CAM_C`) and share enough textured
overlap. It matches features across the four cameras of a device pair, triangulates them metrically in each device via
its factory stereo baseline, and aligns the two metric point clouds. An explicit `setKnownDistance()` always takes
precedence, and a scene that is too weakly constrained is reported as unobservable rather than guessed. Turn it off
with:

```python
calibration.setEstimateInterDeviceScale(False)
```

With it off (or when the scene is unobservable), the inter-device translation keeps the magnitude of the initial
guess — the classic scale-free result — so you would fall back to a stereo pair per device or a `setKnownDistance()`.

---

## 5) Stitch replay — the bird's-eye view

Project the recording onto the floor plane stored in `session.json`. Build it from the ground-truth rig and from the
estimated rig and compare — both should show the floor grid, the red circle and the blue diagonal running continuously
across the three camera fans (only content *above* the floor doubles, which is expected for a planar projection):

```bash
python3 multi_device_replay.py -i rec_synth -m stitch --rig rig.json           -o bev_ground_truth.png
python3 multi_device_replay.py -i rec_synth -m stitch --rig rig_estimated.json -o bev_estimated.png
```

Drop `-o` to open a live preview instead (needs `opencv-python`, press `q` to quit).

---

## 6) Rig compare — estimated vs. ground truth

Score the estimated rig against the exact one (the reference/truth rig comes first, the estimate second):

```bash
python3 multi_device_rig_compare.py rec_synth/rig.json rig_estimated.json
```

Expected on the default cooperative rig (`distance` reads *truth → estimate*):

```text
expressed in SYNTH0:CAM_B
SYNTH1:CAM_B: rotation   0.02 deg, translation    0.1 cm, distance   78.3 ->   78.4 cm
SYNTH2:CAM_B: rotation   0.01 deg, translation    0.5 cm, distance  155.3 ->  154.8 cm
```

---

## 7) The hard case — an unobservable rig

Widening the azimuth spread reproduces a real wide-baseline rig where neighbouring cameras share only a small part of
their field of view, on data whose answer is still known exactly:

```bash
python3 multi_device_synthetic_record.py -o rec_wide --azimuth -50 0 50
python3 multi_device_replay.py -i rec_wide -m calibrate --rig rig_guess.json --samples 16 \
    --performance-mode RELAXED_COVERAGE -o rig_wide.json
```

Here the estimation hits the coverage/observability wall: it either fails the coverage check (`passed=False`,
`info` explains why — relax it with `--performance-mode`), or the estimator reports the metric scale as unobservable
rather than inventing one. That is the correct behaviour — the fix for such a rig is more shared overlap (sweep a
textured target through the common volume) or a measured `setKnownDistance()`, not a looser threshold.

---

## Troubleshooting

| Symptom | Cause / fix |
| --- | --- |
| `ModuleNotFoundError: depthai` | `PYTHONPATH` does not point at `build/bindings/python`, or the module was not built |
| `AttributeError: … MultiDeviceCalibration` / `Stitching` | The module was built without `DEPTHAI_DYNAMIC_CALIBRATION_SUPPORT=ON` |
| Calibrate replay is very slow | ASIFT over many samples — lower `--samples`, or `setEstimateInterDeviceScale(False)` to skip scale recovery |
| `could not recover the metric scale … (unobservable)` | Too little shared, well-triangulated overlap — expected for a wide spread (step 7); use a closer/more-textured shared volume or `setKnownDistance()` |
| Floor grid doubles in the stitch | The rig is wrong (re-run calibrate), or you are looking at content *above* the plane, which always doubles in a planar projection |
