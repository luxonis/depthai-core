# Multi-device (Python) — DepthAI

One `dai.Pipeline` can drive several devices at once. This folder shows how to build such a pipeline, how to calibrate
the devices against each other, and how to fuse their streams into a single image — either a **panorama** (registered
from image content) or a **bird's-eye view** (a calibrated projection onto a plane).

> Press **`q`** in the preview window to quit any of the examples.

| Script | What it shows |
| --- | --- |
| [`multi_device_pipeline.py`](multi_device_pipeline.py) | Several devices in one pipeline, synced, with the coordinate frame of every stream |
| [`multi_device_calibration.py`](multi_device_calibration.py) | Estimating the rig — the poses *between* the devices — and saving it as json |
| [`multi_device_stitching.py`](multi_device_stitching.py) | Panorama stitching, no calibration needed |
| [`multi_device_planar_stitching.py`](multi_device_planar_stitching.py) | Bird's-eye view: calibrated projection onto a plane |
| [`planar_stitching_synthetic.py`](planar_stitching_synthetic.py) | The same bird's-eye view on a synthetic rig, so it runs without any hardware |
| [`multi_device_record.py`](multi_device_record.py) | Record the synced streams of a rig, with every device's calibration and the rig, into one self-contained folder |
| [`multi_device_replay.py`](multi_device_replay.py) | Replay such a recording with **no devices connected** — through calibration or bird's-eye view |
| [`multi_device_cam_sync.py`](multi_device_cam_sync.py), [`multi_device_frame_sync.py`](multi_device_frame_sync.py) | Frame level syncing of two devices |

---

## Concepts

### One pipeline, several devices

Pass the device to `pipeline.create()` and the node runs on it. `createImplicitDevice=False` stops the pipeline from
opening a device of its own:

```python
devices = [dai.Device(dai.DeviceInfo("10.12.228.177")), dai.Device(dai.DeviceInfo("10.12.228.137"))]

with dai.Pipeline(createImplicitDevice=False) as pipeline:
    for device in devices:
        camera = pipeline.create(dai.node.Camera, device).build(dai.CameraBoardSocket.CAM_B)
```

Host nodes (`Sync`, `Stitching`, `CoordinateFrameTransform`, `MultiDeviceCalibration`) see the streams of all devices.
Devices are not hardware synced, so give the syncing node a threshold of a frame interval or two.

### Coordinate frames

`dai.CoordinateFrame(deviceId, socket)` is the identity of a reference frame: one camera on one device. It is what
makes `CAM_B` of two devices distinguishable. Every message carries the frame its extrinsics are relative to:

```python
frame.getTransformation().getExtrinsics().getReferenceFrame()  # e.g. 3604808376:CAM_B
```

The axes are the usual depthai camera axes: **X right, Y down, Z forward** (into the scene), lengths in centimeters
unless a `dai.LengthUnit` says otherwise.

### Per-device calibration vs. the rig

* **Per-device calibration** — intrinsics, distortion and the poses between the cameras of one device. Factory
  calibrated, stored in the device, read automatically.
* **Rig calibration** — the poses *between* devices. Depends on how you mounted them, so it has to be measured on
  site. It is held by `dai.MultiDeviceCalibrationHandler` as a forest of `RigEdge`s and given to the pipeline:

```python
pipeline.setMultiDeviceCalibration(dai.MultiDeviceCalibrationHandler(Path("rig_calibration.json")))
```

Only inter-device edges belong in the rig; anything inside a device always comes from the live device. The graph must
be a forest (no cycles) and may be disconnected — frames in different components simply have no known relation.

```json
{
  "aliases": [],
  "edges": [
    {
      "from": { "deviceId": "2066809955", "socket": 1 },
      "to":   { "deviceId": "3604808376", "socket": 1 },
      "transform": {
        "rotationMatrix": [[...], [...], [...]],
        "translation": { "x": -121.4, "y": 8.7, "z": 96.2 },
        "toDeviceId": "3604808376",
        "toCameraSocket": 1,
        "lengthUnit": 1
      },
      "source": "dynamic-calibration",
      "timestamp": 0
    }
  ]
}
```

`aliases` lets you write a rig against logical names (`"left-post"`) and bind them to device ids at build time.

### Bringing the streams into one frame

`CoordinateFrameTransform` rewrites the extrinsics of the messages passing through it into a single target frame,
using the rig. It is **metadata only** — pixels, intrinsics and distortion are untouched (warping pixels into another
camera's view is `ImageAlign`'s job):

```python
unified = pipeline.create(dai.node.CoordinateFrameTransform).build(outputs, reference)
stitching = pipeline.create(dai.node.Stitching).build([unified.outputs[f"output{i}"] for i in range(len(outputs))])
```

A message whose frame cannot be resolved to the target is a configuration error and throws — usually a missing rig
edge or a device id that never made it into the rig.

---

## 1) Multi-device pipeline

**Script:** `multi_device_pipeline.py`

```bash
python3 multi_device_pipeline.py -d 10.12.228.177 -d 10.12.228.137 -s CAM_B -s CAM_C
```

Opens the given devices, streams the given sockets of each of them, groups the frames with `Sync` and labels every
tile with the coordinate frame it came from. This is the skeleton every other example here is built on.

---

## 2) Rig calibration

**Script:** `multi_device_calibration.py`

```bash
python3 multi_device_calibration.py \
    -d 10.12.228.177 -d 10.12.228.137 \
    -g 35 -120 0 90 \
    -o rig_calibration.json
```

`dai.node.MultiDeviceCalibration` syncs the registered streams, runs the dynamic calibration library over them and
emits the inter-device edges as a `MultiDeviceCalibrationResult`. Two inputs cannot come from the images alone:

* **an initial guess** (`setInitialGuess(from, to, extrinsics)`) — roughly where each device sits w.r.t. the first
  one. Getting it wrong by more than a few tens of degrees usually makes the estimation fail.
* **the metric scale** — the translation between cameras of different devices is only observable up to scale.
  Registering the **stereo pair** of every device (`CAM_B` + `CAM_C`, as the example does) is enough, because their
  baselines are known. On devices without a pair, measure one distance and declare it:
  `setKnownDistance(frameA, frameB, 82.5)`.

Requirements for a good estimate: all devices see a **shared, static, textured** scene, with real overlap between the
views of neighbouring devices; exposure settled; no motion blur.

```text
success (data confidence 0.812)
  2066809955:CAM_B -> 3604808376:CAM_B: [-121.4, 8.7, 96.2] cm
Rig written to rig_calibration.json
```

`result.passed` is `False` when the rig could not be estimated — `result.info` says why (too few features, an edge
without a scale constraint, …). `setContinuous(True)` keeps re-estimating instead of stopping after the first result.

#### Coverage and the performance mode

Before estimating, the library checks that the matched features **cover** enough of each camera's field of view: it
lays an 8×6 grid over the whole (normalized) image and requires the mean fraction of occupied cells to clear a
threshold. Cameras that look at the scene from very different angles only match features in the small region they
share, so most cells stay empty and the estimate is rejected with **"Not enough coverage"** — even though the scene
*is* shared. `setPerformanceMode()` selects how strict this is:

| Mode | mean-coverage ≥ | features ≥ | Use when |
| --- | --- | --- | --- |
| `OPTIMIZE_PERFORMANCE` | 75 % | 4000 | Best accuracy, cooperative scene |
| `DEFAULT` | 70 % | 2000 | Normal, well-overlapping rigs |
| `STATIC_SCENERY` | 50 % | 1500 | Static scene, moderate overlap |
| `OPTIMIZE_SPEED` | 60 % | 800 | Faster, fewer features |
| `RELAXED_COVERAGE` | 30 % | 2000 | Small shared FOV, but still needs many features |
| `SKIP_CHECKS` | 0 % | 0 | Force an estimate regardless of coverage |

```python
calibration.setPerformanceMode(dai.DynamicCalibrationControl.PerformanceMode.SKIP_CHECKS)
```

For a rig whose cameras share only a small part of their view, `SKIP_CHECKS` is what lets the estimation run at all;
mind that with little overlap the result is only as good as the geometry allows, and the inter-device **scale** is
still unobserved unless a stereo pair (CAM_B + CAM_C) is registered per device or a `setKnownDistance()` is given.
The real fix for a poor estimate is more overlap: sweep a textured target across the volume the cameras share.

---

## 3) Panorama stitching

**Script:** `multi_device_stitching.py`

```bash
python3 multi_device_stitching.py -d 10.12.228.177 -d 10.12.228.137 -m CYLINDRICAL
```

`Mode.PANORAMA` (the default) wraps OpenCV's stitcher: features are detected in every image, matched pairwise, the
camera rotations are bundle-adjusted, and the images are warped onto a common surface and blended. Consequences:

* **no calibration is needed**, but neighbouring cameras must **overlap** (~20% is a good minimum) and see enough
  texture,
* only **rotation** between the cameras is modelled, so the scene should be far compared to the distance between the
  cameras — otherwise parallax shows up as ghosting,
* `setCameraModel()` picks the surface: `SPHERICAL` (default), `CYLINDRICAL` for wide horizontal panoramas,
  `PINHOLE` for narrow fields of view.

Registration runs on the first `setEstimationFrames()` groups and the result is then reused; `setContinuous(True)`
re-registers on every group, which is much slower but tolerates cameras moving w.r.t. each other. Everything else —
`setFeaturesFinder`, `setFeaturesMatcher`, `setBundleAdjuster`, `setExposureCompensator`, `setSeamFinder`,
`setBlender`, `setWaveCorrection`, the working resolutions — maps directly onto the OpenCV stitching pipeline.

---

## 4) Bird's-eye view (planar projection)

**Scripts:** `multi_device_planar_stitching.py`, `planar_stitching_synthetic.py`

```bash
# synthetic rig, no hardware needed
python3 planar_stitching_synthetic.py
python3 planar_stitching_synthetic.py --top-down

# real rig
python3 multi_device_planar_stitching.py \
    -c rig_calibration.json \
    -d 10.12.228.177 -d 10.12.228.137 -d 10.12.228.157 -s CAM_B \
    --plane-point 0 250 0 --plane-normal 0 -1 0
```

`Mode.PLANAR_PROJECTION` ignores image content entirely and uses the calibration carried by the messages:

1. for every pixel of the virtual camera, cast a ray into the common reference frame,
2. intersect it with the configured plane,
3. project that 3D point back into every source camera (distortion included),
4. blend the contributions.

Because nothing is registered from the images, **overlap is not required** — cameras that see disjoint parts of the
plane simply paint disjoint parts of the output. What *is* required is that all inputs are expressed in the same
reference frame, i.e. a `CoordinateFrameTransform` in front of the node (and therefore a rig calibration).

The maps, seams and exposure gains are computed once, from the first synced group; after that a frame costs a
`remap` + blend. `resetTransform()` throws that setup away and rebuilds it from the next group.

### The plane

`setPlane(point, normal, unit=CENTIMETER)`, both in the **reference frame**. For a rig looking down at the floor from
2.5 m, with the reference camera level, the floor is 250 cm below the camera and its normal points up, i.e. against
the camera's Y axis:

```python
stitching.setPlane(dai.Point3f(0, 250, 0), dai.Point3f(0, -1, 0))
```

The plane does not have to be horizontal — a wall, a conveyor belt or a tilted table work the same way. The normal
does not have to be normalized, and its sign does not matter. Cameras looking at the plane from the other side, or
edge-on, contribute nothing.

### The view

Left alone, the node **computes the view from the content**: it intersects every camera's field of view with the
plane, frames all footprints, and picks a resolution close to the input sampling, bounded by `setMaxViewSize()`. That
is usually what you want for an overview.

To place the virtual camera yourself:

```python
view = dai.node.Stitching.VirtualCamera.lookAt(
    position=dai.Point3f(0, -50, 200),   # camera center, in the reference frame
    target=dai.Point3f(0, 250, 200),     # point on the plane it looks at
    up=dai.Point3f(0, 0, 1),             # direction that ends up pointing up in the image
    hFovDegrees=100.0,
    width=1400,
    height=1400,
)
stitching.setView(view)
```

`setViewAuto()` goes back to the automatic view. `VirtualCamera` is a plain pinhole camera — you can also fill
`pose`, `intrinsics`, `width` and `height` directly, e.g. to reuse the intrinsics of a real camera.

### Tuning

| Setting | Meaning | Default |
| --- | --- | --- |
| `setMaxRange(cm)` | Distance from a camera beyond which the plane is not painted. Also cuts off the horizon, where a handful of pixels would be stretched over half the plane | 1000 cm |
| `setMinIncidenceAngle(deg)` | Rays hitting the plane at a shallower angle are dropped as too stretched | 5° |
| `setMaxViewSize(w, h)` | Upper bound on the automatic view | 1920×1920 |
| `setExposureCompensator`, `setSeamFinder`, `setBlender`, `setInterpolation` | Same compositing knobs as the panorama mode | `GAIN_BLOCKS`, `GRAPHCUT_COLOR`, `MULTI_BAND`, `LINEAR` |

### The output

A `BGR888i` `ImgFrame` whose `ImgTransformation` describes the virtual pinhole camera in the reference frame, so the
result is a normal calibrated image: you can project detections made in the bird's-eye view back onto the plane, or
feed it to any node that expects intrinsics and extrinsics.

```python
projected = projectedQueue.get()
transformation = projected.getTransformation()
intrinsics = transformation.getIntrinsicMatrix()
pose = transformation.getExtrinsics().getTransformationMatrix()  # virtual camera -> reference frame
```

---

## 5) Record & replay

**Scripts:** `multi_device_record.py`, `multi_device_replay.py`

A multi-device rig is awkward to keep plugged in, so record it once and iterate offline. The host nodes
(`MultiDeviceCalibration`, `CoordinateFrameTransform`, `Stitching`) run the same whether their frames come from live
cameras or from `ReplayVideo`, so a recording can be fed straight back through them.

```bash
# record ~7 s of CAM_B + CAM_C of three devices, plus a rig and the stitching plane to store with it
python3 multi_device_record.py -o rec \
    -d 10.12.228.177 -d 10.12.228.137 -d 10.12.228.157 -s CAM_B -s CAM_C \
    -t 7 -c rig_calibration.json --plane-point -10.8 124.5 49.3 --plane-normal -0.087 0.929 0.359

# bird's-eye view of the recording, no hardware
python3 multi_device_replay.py -i rec -m stitch -o rec_bev.png

# rig estimation on the recording, no hardware
python3 multi_device_replay.py -i rec -m calibrate --samples 8 -o rig_estimated.json
```

The recording is **self-contained**: besides the per-camera video (`.avi`) and metadata (`.mcap`) it stores every
device's factory calibration (`<deviceId>_calibration.json`), the rig (`rig.json`, when given) and a manifest
(`session.json`, which also carries the stitching plane when `--plane-point/--plane-normal` are given, so the replay
reproduces the same view without re-specifying it). That is everything the geometry needs, so nothing is read from a
live device on replay:

* `MultiDeviceCalibration.setDeviceCalibration(deviceId, calibration)` supplies the per-device calibration that would
  otherwise be read from the assigned device — the one thing that used to force a live device into a replay pipeline.
* `CoordinateFrameTransform` needs both the inter-device rig (stored) and each device's intra-device edges; the
  replay script completes the stored rig with those edges from the saved per-device calibrations, exactly as the node
  does from live devices.

Record and replay each work with **stock 3.8.0 firmware** on the devices (see the compatibility note in the blog
post): the reference frame's device id is stamped on the host, so it survives into the recording.

> Independent devices are not hardware-synced, so their recorded timestamps are tens to hundreds of ms apart. The
> replay groups them loosely (a generous `setSyncThreshold`); for a static calibration/stitching scene that is fine.

---

## Troubleshooting

| Symptom | Cause |
| --- | --- |
| No output at all | The sync threshold is too tight for free-running cameras — `setSyncThreshold(timedelta(seconds=2.0 / fps))` |
| `No transformation path between … they belong to different connected components` | The rig has no path from that camera to the target frame: a missing edge, or a device id in the rig that does not match the live device |
| `CoordinateFrameTransform requires a multi-device calibration` | `pipeline.setMultiDeviceCalibration()` was never called |
| `PLANAR_PROJECTION mode needs all inputs expressed in the same reference frame` | `CoordinateFrameTransform` is missing in front of the node, or one input bypasses it |
| `None of the … inputs sees the plane` | The cameras look at the plane from the wrong side, the plane is farther than `setMaxRange()`, or every ray hits it below `setMinIncidenceAngle()` |
| Planar mode: everything on the plane lines up, objects above it are doubled | Expected — only the plane is reconstructed; anything off it is seen from different directions by different cameras |
| Planar mode: the plane itself is misaligned between cameras | The rig calibration is off; re-run `multi_device_calibration.py` with a better initial guess and a richer shared scene |
| Panorama: ghosting or a dropped camera | Not enough overlap or texture, or too much parallax for a rotation-only model — use the planar mode with a calibrated rig instead |
