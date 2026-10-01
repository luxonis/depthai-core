# Multi-device calibration — DepthAI

This README covers **`multi_device_calibration.py`** and its C++ counterpart
`examples/cpp/Misc/MultiDevice/multi_device_calibration.cpp`. Both estimate the
metric pose between two or more DepthAI devices using the experimental
**`dai.beta.node.MultiDeviceCalibration`** node and save the result as JSON.

The other scripts in this folder (frame sync, host nodes, device-to-device
relay, system logger) are independent multi-device examples and are not
described here.

---

## What the node does

`MultiDeviceCalibration` is a **host-only, one-shot** node. You register one or
more camera streams per device, send a `start` command, and the node:

1. Waits for synchronized image groups that contain a frame from **every**
   registered camera (an internal `Sync` node does the grouping).
2. Loads `sampleCount` such groups into the Dynamic Calibration Library (DCL).
3. Solves for the rigid transform between the devices and emits **one**
   `MultiDeviceCalibrationResult`.

The result is pure data. The node never writes anything to a device. You decide
whether to save it, or hand it to the pipeline with
`pipeline.setMultiDeviceCalibration(...)`.

### Requirements

- At least **two devices**, each with at least one registered camera.
- A `depthai` build with **beta**, **dynamic calibration** and **OpenCV**
  support (`DEPTHAI_BUILD_BETA`, `DEPTHAI_DYNAMIC_CALIBRATION_SUPPORT` and
  `DEPTHAI_OPENCV_SUPPORT`; the first two default to ON).
- A source of **metric scale**: either a factory-calibrated stereo pair on at
  least one device (`setStereoPair`) or a measured camera-to-camera distance
  across devices (`setKnownDistance`).
- All cameras must see the **same static, textured scene** while samples are
  collected. Keep the devices still.

---

## Run

```bash
# First two devices found, 10 samples, output next to the script
python3 multi_device_calibration.py

# Explicit devices (IDs or IPs), more samples, custom output path
python3 multi_device_calibration.py -d 1944301021AF721300 192.168.1.42 -n 20 -o rig.json
```

| Option | Default | Meaning |
| --- | --- | --- |
| `-d`, `--devices` | first two available | Device IDs or IPs, at least two |
| `-n`, `--sample-count` | `10` | Synchronized groups to collect before solving |
| `-o`, `--output` | `multi_device_calibration.json` | Where to save the calibration |

The C++ binary accepts the same options:

```bash
./multi_device_calibration -d <id_or_ip> <id_or_ip> [-n 20] [-o rig.json]
```

**Example console output:**

```
Using device 14442C1081A6BCD600
Using device 1944301021AF721300
Point all devices at the same textured scene and keep them still.
Calibration saved to multi_device_calibration.json
Confidence: 0.912
Sampson error: 0.4133
```

If the solver cannot find a consistent pose, the script raises with the
node's `info` text, for example
`DynamicCalibration failed: Multi-device calibration optimization failed`.

---

## Flow of the example

```python
with dai.Pipeline(createImplicitDevice=False) as pipeline:
    calibration = pipeline.create(dai.beta.node.MultiDeviceCalibration)
    calibration.setSampleCount(args.sample_count)
    calibration.sync.setSyncThreshold(timedelta(seconds=5))

    for info in deviceInfos:
        device = pipeline.addDevice(info)
        deviceId = device.getDeviceId()
        for socket in (dai.CameraBoardSocket.CAM_B, dai.CameraBoardSocket.CAM_C):
            camera = pipeline.create(dai.node.Camera, device).build(socket, sensorFps=5)
            calibration.addCamera(deviceId, socket, camera.requestFullResolutionOutput(fps=5))
        calibration.setStereoPair(deviceId, dai.CameraBoardSocket.CAM_B, dai.CameraBoardSocket.CAM_C)

    controlQueue = calibration.inputControl.createInputQueue()
    resultQueue = calibration.calibrationOutput.createOutputQueue()

    pipeline.start()
    controlQueue.send(dai.beta.MultiDeviceCalibrationControl.start())
    result = resultQueue.get(timedelta(minutes=3))

    if result is None or not result.passed or result.graph is None:
        raise RuntimeError(result.info if result is not None else "Calibration timed out")
    result.getHandler().toJsonFile(args.output)
```

1. One pipeline, no implicit device. Every device is added explicitly with
   `pipeline.addDevice`, and each `Camera` is created on its own device.
2. `CAM_B` and `CAM_C` of every device are registered. Full-resolution frames
   give the solver the most features; 5 fps keeps the host load low.
3. The devices' factory `CAM_B`/`CAM_C` pair provides metric scale.
4. The sync threshold is generous (5 s) because the devices are not hardware
   synchronized and the scene is static. Tighten it if the scene moves, or use
   FSYNC/PTP as shown in `multi_device_frame_sync.py`.
5. `start()` begins collection. The first complete group initializes the
   solver; after `sampleCount` groups the result is emitted.

---

## Pipeline diagram

```
Device A  CAM_B ──▶ [Camera] ──▶ MultiDeviceCalibration.inputs["camera_A_CAM_B"] ┐
          CAM_C ──▶ [Camera] ──▶ MultiDeviceCalibration.inputs["camera_A_CAM_C"] │
                                                                                 ├─▶ (internal Sync) ─▶ DCL ─▶ calibrationOutput
Device B  CAM_B ──▶ [Camera] ──▶ MultiDeviceCalibration.inputs["camera_B_CAM_B"] │
          CAM_C ──▶ [Camera] ──▶ MultiDeviceCalibration.inputs["camera_B_CAM_C"] ┘

inputControl ◀── MultiDeviceCalibrationControl.start() / stop() / reset()
```

`addCamera` creates and links the input for you; the names above are only
relevant if you inspect the sync node directly.

---

## API reference

### `dai.beta.node.MultiDeviceCalibration`

All configuration methods must be called while the node is **idle**: before the
first `start`, or after a `reset`. They raise otherwise.

| Member | Description |
| --- | --- |
| `inputControl` | Input for `MultiDeviceCalibrationControl` messages. |
| `calibrationOutput` | Emits exactly one `MultiDeviceCalibrationResult` per run. |
| `sync` | The internal `dai.node.Sync`. Use `sync.setSyncThreshold(...)` to control how close in time the frames of one group must be. |
| `addCamera(deviceId, socket, cameraOutput)` | Register a camera stream. The device ID and socket are explicit because frame metadata is not a reliable cross-device identity. Each (device, socket) pair may be registered once. |
| `setSampleCount(n)` / `getSampleCount()` | Number of complete synchronized groups to collect before solving. Default `10`, minimum `1`. |
| `setStereoPair(deviceId, leftSocket, rightSocket)` | Restrict metric scale recovery to this factory-calibrated pair on `deviceId`. Both sockets must be registered cameras. One pair per device. |
| `setKnownDistance(fromDeviceId, fromSocket, toDeviceId, toSocket, distance, unit=CENTIMETER)` | Supply a measured camera-center distance between two cameras on **different** devices. Alternative or complement to a stereo pair for metric scale. |
| `setInitialGuess(fromDeviceId, fromSocket, toDeviceId, toSocket, guess)` | Optional starting pose from one device's **local calibration origin** to another's. `guess` is a `dai.Extrinsics` whose `toDeviceId`/`toCameraSocket` must match the explicit destination. Only one direction per device pair. |
| `setDeviceCalibration(deviceId, calibrationHandler)` | Override the live device calibration. Needed for replayed or recorded streams and tests; live devices are read automatically. |

**Local calibration origin.** Each device's intrinsics and extrinsics come from
its own `CalibrationHandler`. The node resolves every registered camera to that
handler's origin socket, the camera at the root of the device's extrinsics
chain (`CAM_A` for the devices in the sample output below). All cross-device edges, initial
guesses and the output graph are expressed between these origins, not between
the sockets you registered.

**Reference device.** The device with the lexicographically lowest device ID is
the reference; all other devices receive an edge towards it.

### `dai.beta.MultiDeviceCalibrationControl`

| Command | Effect |
| --- | --- |
| `start()` | Begin a run. Ignored unless the node is idle. |
| `stop()` | Abort a run in progress. Emits a result with `passed == False` and `info == "stopped"`, then returns to idle. |
| `reset()` | Clear the finished or aborted run and return to idle, so configuration can change and `start()` can be sent again. |

The equivalent constructor form is
`dai.beta.MultiDeviceCalibrationControl(dai.beta.MultiDeviceCalibrationControl.Commands.Start())`.

Lifecycle: `idle ──start──▶ collecting ──(samples reached / failure)──▶ complete ──reset──▶ idle`.
A `stop` during `collecting` goes straight back to `idle`.

### `dai.beta.MultiDeviceCalibrationResult`

| Field | Type | Meaning |
| --- | --- | --- |
| `passed` | `bool` | `True` when the solver succeeded and returned a pose for every device. |
| `graph` | `list[dai.MultiDeviceExtrinsics] \| None` | Meter-normalized edges from each device's local origin to the reference device's origin. Present only when `passed`. |
| `dataConfidence` | `float` | Solver's quality score of the collected samples, 0.0 to 1.0. |
| `sampsonError` | `float` | Sampson error of the estimated calibration over the samples. Lower is better. |
| `info` | `str` | Why the run did not pass. Empty on success. |
| `getHandler()` | `MultiDeviceCalibrationHandler \| None` | Wraps `graph` for saving and querying. |

### `dai.beta.MultiDeviceCalibrationHandler`

| Member | Description |
| --- | --- |
| `MultiDeviceCalibrationHandler(graph)` / `(path)` / `fromJson(dict)` | Construct from edges, a JSON file, or a parsed JSON object. The graph is validated. |
| `toJson()` / `toJsonFile(path)` | Serialize with translations in **centimeters**. |
| `getGraph()` | The validated, **meter**-normalized edges. |
| `getDeviceSocket(deviceId)` | The local origin socket a device uses in the graph, or `None`. |
| `getExtrinsicsToOrigin(deviceId, localOriginSocket)` | Transform from that device's local origin to the common origin of its connected component, in meters. The common origin is the lowest (device ID, socket) pair in the component. |

### Using the result in a pipeline

```python
handler = dai.beta.MultiDeviceCalibrationHandler("multi_device_calibration.json")
pipeline.setMultiDeviceCalibration(handler.getGraph())   # validates and pushes the graph to the devices
graph = pipeline.getMultiDeviceCalibration()             # list of edges or None
pipeline.clearMultiDeviceCalibration()
```

`setMultiDeviceCalibration` validates the graph against each device's own
calibration and raises if they disagree. On a running pipeline the graph is
pushed to every device, so streaming `Camera` and `ToF` nodes switch to it.
Devices updated before a failure are rolled back.

---

## Output file

`toJsonFile` writes one object with a `graph` list. Each entry is a directed
edge; this one maps device `232264861` to device `1531764865`, both via their
`CAM_A` origins (`0`), with translation in centimeters (`lengthUnit: 1`):

```json
{
  "graph": [
    {
      "fromDeviceId": "232264861",
      "fromSocket": 0,
      "extrinsics": {
        "toDeviceId": "1531764865",
        "toCameraSocket": 0,
        "lengthUnit": 1,
        "rotationMatrix": [[0.688, -0.007, 0.726], [-0.008, 1.000, 0.017], [-0.726, -0.017, 0.688]],
        "translation": {"x": -223.2, "y": 1.08, "z": 67.9},
        "specTranslation": {"x": 0.0, "y": 0.0, "z": 0.0}
      }
    }
  ]
}
```

---

## Troubleshooting

- **`At least two devices are required`** — fewer than two devices were
  discovered. Pass them explicitly with `-d`.
- **`Camera socket not found on the connected device`** — a selected device has
  no `CAM_B`/`CAM_C`. Pick devices with a stereo pair or change the sockets and
  stereo pair in the script.
- **No result within the timeout** — groups never complete. Check that every
  camera streams, raise the sync threshold, or lower the fps.
- **`DynamicCalibration failed: ... optimization failed`** — the views do not
  overlap enough or the scene lacks texture. Point all devices at the same
  richly textured area, add light, and increase `--sample-count`.
- **`no explicit or live calibration is available for device ...`** — the
  stream comes from a replay or a device without calibration. Provide one with
  `setDeviceCalibration`.
- **Configuration call raises "cannot be changed unless the node is idle"** —
  send `reset()` before reconfiguring after a finished run.
