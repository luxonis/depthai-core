# DepthAI Examples

For more information about the examples, please refer to the [DepthAI documentation](https://stg.docs.luxonis.com/software/v3/examples/).
## Supported platforms
The examples are now split into three categories:
* In the root directory of examples, there are the examples that are supported on both RVC2 and RVC4 platforms devices
* In RVC2/RVC4 directories there are the examples that are supported only on one of the platforms

## Python

To get the python examples running, install the requirements with:

```
python3 depthai-core/examples/python/install_requirements.py
```
and run the example with:
```
python3 depthai-core/examples/python/Camera/camera_output.py
```

## C++

To build the examples configure with following option added from the root of the repository:
```
cmake -S. -Bbuild -D'DEPTHAI_BUILD_EXAMPLES=ON'
cmake --build build
```

Then navigate to `build/examples` folder and run a preferred example
```
cd build/examples
./cpp/Camera/camera_output
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
**NOTE** Currently, Rerun does not work with Numpy 2.0, you need to downgrade it to, for example 1.24.4 to be able to properly view images.

> ℹ️ Multi-Config generators (like Visual Studio on Windows) will have the examples built in `build/examples/Camera/[Debug/Release/...]/camera_output`
