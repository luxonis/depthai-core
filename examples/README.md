# Examples for the Depthai V3 API

The examples in this directory show the existing functionality of the Depthai V3 API.

The examples range from the ones that were just minimally ported from the V2 API, to showcase that porting the existing code is straightforward,
to the ones that are specifically designed to show the new features of the V3 API.

The StereoDepth/stereo_autocreate.py example is a good example of the new features of the V3 API which showcases the ability to automatically create the stereo inputs
as well as the ability to create a custom node that can be used in the pipeline.

```python
# Create pipeline
with dai.Pipeline() as pipeline:
    # Allow stereo inputs to be created automatically
    stereo = pipeline.create(dai.node.StereoDepth).build(autoCreateCameras=True)
    # This can be alternatively written as:
    # stereo = dai.node.StereoDepth(autoCreateCameras=True)
    visualizer = pipeline.create(StereoVisualizer).build(stereo.disparity)
    pipeline.start()
    while pipeline.isRunning():
        time.sleep(0.1)
```

## Supported platforms
The examples are now split into three categories:
* In the root directory of examples, there are the examples that are supported on all platforms
* In RVC2/RVC4 directories there are the examples that are supported only on the RVC2/RVC4 platform

In the future we plan to make the examples more platform agnostic and we'll be slowly moving as many examples as possible to the root directory.

## Supported languages
The examples are currently in Python and C++. The C++ examples are in the `cpp` directory and the Python examples are in the `python` directory.

Currently there are more python examples than C++ examples, but we plan to match the examples in both languages in the future and keep them 1:1.

## Syntax differences compared to the V2 API

In Python, currently, there exist two syntaxes for creating nodes. Before we decide, we first want to gather feedback from you.

One syntax uses the `Pipeline.create(...)` and `Node.build(...)` methods like this `pipeline.create(dai.node.StereoDepth).build(autoCreateCameras=True)`.

The other syntax uses the pipeline from the `with` statement and does all the work in the constructor -- e.g. `dai.node.StereoDepth(autoCreateCameras=True`.

In Python, both syntaxes also accept keyword arguments that mirror setter methods -- e.g. `dai.node.VideoEncoder(bitrate=2*24)`. Currently, not all parameters are available.

We'd love to hear your opinion which one you prefer. Currently, this syntax is now available only for `Camera`, `DetectionNetwork`, `StereoDepth` and `VideoEncoder`. Other nodes still have to be created with `pipeline.create`, parameters set with setter methods and inputs linked with `.link`.

Examples of this can be found in the `VideoEncoder` example.

## Python installation

To get the examples running, install the requirements with:

```
python3 depthai-core/examples/python/install_requirements.py
```

NOTE: Right now wheels for windows are missing, but wheels for MacOS and Linux, both x86_64 and arm64 are available.

## What's new in the V3 API
* No more expliclit XLink nodes - the XLink "bridges" are created automatically
* Host nodes - nodes that run on the host machine now cleanly interoperate with the device nodes
* Custom host nodes - the user can create custom nodes that run on the host machine
  * Both `ThreadedHostNode` and `HostNode` are supported.
  * `ThreadedHostNode` works in a very similar fashion to the `ScriptNode` where the user specifes a `run` function which is then executed in a separate thread.
  * `HostNode` has an input map `inputs` where all the inputs are implicitly synced
  * Available both in Python and C++
* Record and replay nodes
  * Holistic record and replay is WIP
* Support for both RVC2&RVC3 with initial support for RVC4
* Device is now available at node construction, so we will be able to create smart defaults
  * Not used extensively yet, will be added gradually to more and more nodes.
* Support for NNArchive for the existing NN nodes
* `build(params)` functions for nodes where they can autocreate its inputs
  * Not yet used extensively yet, will be added gradually to more and more nodes.


## How to port an example from V2 to V3
The process of porting an example from V2 to V3 should be straightforward.

The minimal needed changes:
* Remove the explicit creation of the device **or** pass the device in the pipeline constructor
* Remove the explicit XLink nodes
* Replace any `.getOutputQueue()` calls with `output.createOutputQueue()` calls


### Quick porting example
Let's take the simplest `rgb_video.py` example and port it to the V3 API.

The commented out code from the old API is commented with #ORIG and the new code is commented with #NEW.:
```python
#!/usr/bin/env python3

import cv2
import depthai as dai

# Create pipeline
# ORIG
# pipeline = dai.Pipeline()
with dai.Pipeline() as pipeline:
    # Define source and output
    camRgb = pipeline.create(dai.node.ColorCamera)

    # ORIG
    # xoutVideo = pipeline.create(dai.node.XLinkOut)
    # xoutVideo.setStreamName("video")

    # Properties
    camRgb.setBoardSocket(dai.CameraBoardSocket.CAM_A)
    camRgb.setResolution(dai.ColorCameraProperties.SensorResolution.THE_1080_P)
    camRgb.setVideoSize(1920, 1080)

    # Linking
    camRgb.video.link(xoutVideo.input)
    # NEW
    videoQueue = camRgb.video.createOutputQueue()

# ORIG
# with dai.Device(pipeline) as device:
#   video = device.getOutputQueue(name="video", maxSize=1, blocking=False)
#   while True:
# NEW
    while pipeline.isRunning():
        videoIn = video.get()
        # Get BGR frame from NV12 encoded video frame to show with opencv
        # Visualizing the frame on slower hosts might have overhead
        cv2.imshow("video", videoIn.getCvFrame())

        if cv2.waitKey(1) == ord('q'):
            break
```


## Running examples

To build the examples configure with following option added from the root of the repository:
```
cmake -S. -Bbuild -D'DEPTHAI_BUILD_EXAMPLES=ON'
cmake --build build
```

Then navigate to `build/examples` folder and run a preferred example
```
cd build/examples
./MobileNet/rgb_mobilenet
```

## VSLAM
The VSLAM example is a bit more complex and requires additional dependencies - OpenCV and PCL. To install them, run the following command:
```
sudo apt install -y libopencv-dev libpcl-dev
```

Then, build the examples with the VSLAM example enabled:
```
cmake -S. -Bbuild -D'DEPTHAI_BUILD_EXAMPLES=ON'  -D'DEPTHAI_BASALT_SUPPORT=ON' -D'DEPTHAI_PCL_SUPPORT=ON' -D'DEPTHAI_RTABMAP_SUPPORT=ON'
cmake --build build
```
To run those examples you also need to install Rerun, you can install it automatically with
```
python3 depthai-core/examples/python/install_requirements.py --install_rerun
```
You can also install it separately, installation instructions can be found [here](https://rerun.io/docs/getting-started/installing-viewer). If you use Numpy v2.0 you might need to downgrade it for Rerun.
**NOTE** Currently, Rerun does not work with Numpy 2.0, you need to downgrade it to, for example 1.26.4 to be able to properly view images.

> ℹ️ Multi-Config generators (like Visual Studio on Windows) will have the examples built in `build/examples/MobileNet/[Debug/Release/...]/rgb_mobilenet`
