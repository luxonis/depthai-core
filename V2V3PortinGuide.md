# DepthAI v2 → v3 Porting Guide

This document describes the changes between the v2 and v3 APIs of DepthAI and how to migrate existing code.

> ℹ️ **Tip**: Copy this documents raw Markdown (not the rendered HTML) and paste it into your preferred large-language-model (LLM) alongside the code you want to port. The model usually generates a solid starting point for porting your code.

## What's new in the v3 API

* No more **explicit** XLink nodes – the XLink “bridges” are created automatically.
* Host nodes – nodes that run on the host machine now work cleanly with device‑side nodes.
* Custom host nodes – users can create custom nodes that run on the host machine

  * Both `ThreadedHostNode` and `HostNode` are supported.
  * `ThreadedHostNode` works similarly to `ScriptNode`; the user **specifies** a `run` function that executes in a separate thread.
  * `HostNode` exposes an input map `inputs` whose entries are implicitly synced.
  * Available in both Python and C++.
* Record‑and‑replay nodes.
* `Pipeline` now has a live device that can be queried during pipeline creation.
* Support for the new **Model Zoo**.
* `ImageManip` has a refreshed API with better‑defined behaviour.
* `ColorCamera` and `MonoCamera` are deprecated in favour of the new `Camera` node.

---

## Minimal changes required

* Remove the explicit creation of `dai.Device` (unless you intentionally pass a live device handle via the pipeline constructor – a rare edge case).
* Remove explicit XLink nodes.
* Replace `dai.Device(pipeline)` with `pipeline.start()`.
* Replace any `.getOutputQueue()` calls with `output.createOutputQueue()`.
* Replace any `.getInputQueue()` calls with `input.createInputQueue()`.

---

## Quick port: simple RGB stream example

Below, the old v2 code is commented with `# ORIG` and the new code with `# NEW`.

```python
#!/usr/bin/env python3

import cv2
import depthai as dai

# Create pipeline
pipeline = dai.Pipeline()

# Define source and output
camRgb = pipeline.create(dai.node.ColorCamera)

# ORIG – explicit XLink removed in v3
# xoutVideo = pipeline.create(dai.node.XLinkOut)
# xoutVideo.setStreamName("video")

# Properties
camRgb.setBoardSocket(dai.CameraBoardSocket.CAM_A)
camRgb.setResolution(dai.ColorCameraProperties.SensorResolution.THE_1080_P)
camRgb.setVideoSize(1920, 1080)

# Linking
# ORIG
# camRgb.video.link(xoutVideo.input)
# NEW – output queue straight from the node
videoQueue = camRgb.video.createOutputQueue()

# ORIG – entire `with dai.Device` block removed
# with dai.Device(pipeline) as device:
#   video = device.getOutputQueue(name="video", maxSize=1, blocking=False)
#   while True:
# NEW – start the pipeline
pipeline.start()
while pipeline.isRunning():
    videoIn = videoQueue.get()  # blocking
    cv2.imshow("video", videoIn.getCvFrame())
    if cv2.waitKey(1) == ord('q'):
        break
```

This runs on RVC2 devices. Note that `ColorCamera`/`MonoCamera` nodes are deprecated on RVC4; see the next section for using `Camera` instead.

---

## Porting `ColorCamera` / `MonoCamera` usage to `Camera`

The new `Camera` node can expose as many outputs as you request.

```python
camRgb = pipeline.create(dai.node.ColorCamera)
camRgb.setPreviewSize(300, 300)
camRgb.setInterleaved(False)
camRgb.setColorOrder(dai.ColorCameraProperties.ColorOrder.RGB)
outputQueue = camRgb.preview.createOutputQueue()
```

turns into

```python
camRgb = pipeline.create(dai.node.Camera).build()  # don’t forget .build()
cameraOutput = camRgb.requestOutput((300, 300), type=dai.ImgFrame.Type.RGB888p)  # replaces .preview
outputQueue = cameraOutput.createOutputQueue()
```

Request multiple outputs simply by calling `requestOutput` again. For full‑resolution use‑cases that previously used `.isp`, call `requestFullResolutionOutput()` instead.

For former `MonoCamera` pipelines, replace the `.out` output with `requestOutput`, e.g.

```python
mono = pipeline.create(dai.node.Camera).build()
monoOut = mono.requestOutput((1280, 720), type=dai.ImgFrame.Type.GRAY8)
```

---

## Porting the old `ImageManip` to the new API

The new API tracks every transformation in sequence and separates *how* the final image is resized.
See the [official documentation](https://docs.luxonis.com/software/v3/depthai-components/nodes/image_manip/) for full details.

### v2 example

```python
#!/usr/bin/env python3

import cv2
import depthai as dai

# Create pipeline
pipeline = dai.Pipeline()

camRgb = pipeline.create(dai.node.ColorCamera)
camRgb.setPreviewSize(1000, 500)
camRgb.setInterleaved(False)
maxFrameSize = camRgb.getPreviewHeight() * camRgb.getPreviewWidth() * 3

# In this example we use 2 imageManips for splitting the original 1000x500
# preview frame into 2 500x500 frames
manip1 = pipeline.create(dai.node.ImageManip)
manip1.initialConfig.setCropRect(0, 0, 0.5, 1)
manip1.setMaxOutputFrameSize(maxFrameSize)
camRgb.preview.link(manip1.inputImage)

manip2 = pipeline.create(dai.node.ImageManip)
manip2.initialConfig.setCropRect(0.5, 0, 1, 1)
manip2.setMaxOutputFrameSize(maxFrameSize)
camRgb.preview.link(manip2.inputImage)

xout1 = pipeline.create(dai.node.XLinkOut)
xout1.setStreamName('out1')
manip1.out.link(xout1.input)

xout2 = pipeline.create(dai.node.XLinkOut)
xout2.setStreamName('out2')
manip2.out.link(xout2.input)

# Connect to device and start pipeline
with dai.Device(pipeline) as device:
    # Output queue will be used to get the rgb frames from the output defined above
    q1 = device.getOutputQueue(name="out1", maxSize=4, blocking=False)
    q2 = device.getOutputQueue(name="out2", maxSize=4, blocking=False)

    while True:
        if q1.has():
            cv2.imshow("Tile 1", q1.get().getCvFrame())

        if q2.has():
            cv2.imshow("Tile 2", q2.get().getCvFrame())

        if cv2.waitKey(1) == ord('q'):
            break
```

### v3 equivalent:

```python
#!/usr/bin/env python3

import cv2
import depthai as dai

# Create pipeline
pipeline = dai.Pipeline()

camRgb = pipeline.create(dai.node.Camera).build()
preview = camRgb.requestOutput((1000, 500), type=dai.ImgFrame.Type.RGB888p)

# In this example we use 2 imageManips for splitting the original 1000x500
# preview frame into 2 500x500 frames
manip1 = pipeline.create(dai.node.ImageManip)
manip1.initialConfig.addCrop(0, 0, 500, 500)
preview.link(manip1.inputImage)

manip2 = pipeline.create(dai.node.ImageManip)
manip2.initialConfig.addCrop(500, 0, 500, 500)
preview.link(manip2.inputImage)

q1 = manip1.out.createOutputQueue()
q2 = manip2.out.createOutputQueue()

pipeline.start()
with pipeline:
    while pipeline.isRunning():
        if q1.has():
            cv2.imshow("Tile 1", q1.get().getCvFrame())

        if q2.has():
            cv2.imshow("Tile 2", q2.get().getCvFrame())

        if cv2.waitKey(1) == ord('q'):
            break
```
