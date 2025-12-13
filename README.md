# DepthAI Library

[![Forum](https://img.shields.io/badge/Forum-discuss-orange)](https://discuss.luxonis.com/)
[![Docs](https://img.shields.io/badge/Docs-DepthAI_API-yellow)](https://stg.docs.luxonis.com/software/v3/)
[![License: MIT](https://img.shields.io/badge/License-MIT-green.svg)](https://opensource.org/licenses/MIT)

DepthAI library for interfacing with Luxonis DepthAI hardware. It's written in C++ and offers Python bindings out of the box.

>  **Important — You’re viewing the `v3.x.y` branch.**
>
> * For production projects that still rely on **v2**, check out the
>   [`v2_stable` branch](https://github.com/luxonis/depthai-core/tree/v2_stable).
> * Need to migrate? Follow the step-by-step [v2 → v3 Porting Guide](./V2V3PortinGuide.md).

## Documentation
Documentation is available over at [Luxonis DepthAI API](https://docs.luxonis.com/software-v3/depthai/)

## Examples
Examples for both C++ and Python are available in the `examples` folder. To see how to build and run them see [README.md](./examples/README.md) for more information.
To build the examples in C++ configure with the following option added:
```
cmake -S. -Bbuild -D'DEPTHAI_BUILD_EXAMPLES=ON'
cmake --build build
```

## Dependencies
- CMake >= 3.20
- C/C++17 compiler
- [Linux] libudev >= 1.0.0
- [optional] OpenCV 4 (required if building examples and for record and replay)
- [optional] PCL (required for point cloud example)

To install libudev on Debian based systems (Ubuntu, etc.): `sudo apt install libudev-dev`

To install OpenCV:
MacOS: `brew install opencv`
Linux: `sudo apt install libopencv-dev`
Windows: `choco install opencv`

To install PCL:
MacOS: `brew install pcl`
Linux: `sudo apt install libpcl-dev`

## Using Python bindings
Installing the latest pre-released version of the library can be done with:
```
python3 -m pip install --extra-index-url https://artifacts.luxonis.com/artifactory/luxonis-python-release-local/ --pre -U depthai
```

or by running:
```
python3 examples/python/install_requirements.py on the branch you want to install
```

For more specific information about Python bindings, see [Python README](./bindings/python/README.md).


## Building

Make sure submodules are updated
```
git submodule update --init --recursive
```

Then configure and build

```
cmake -S . -B build
cmake --build build --parallel [num CPU cores]
```
On Windows it's often required to specify the location of the OpenCV installation. In case you used chocolatey to install OpenCV, you can use the following command:

```
cmake -S . -B build -DOpenCV_DIR=C:/tools/opencv/build -DCMAKE_BUILD_TYPE=Release
cmake --build build --config Release --parallel [num CPU cores]
```
> ℹ️ To speed up build times, use `cmake --build build --parallel [num CPU cores]`.

### Dynamic library

To build a dynamic version of the library configure with the following option added
```
cmake -S . -B build -D'BUILD_SHARED_LIBS=ON'
cmake --build build --parallel [num CPU cores]
```

## Installation and Integration
Installation of the DepthAI library is currently only available as a dynamic library. To install the library, use the following command:

```
cmake -S . -B build -D'BUILD_SHARED_LIBS=ON' -D'CMAKE_INSTALL_PREFIX=[path/to/install/dir]'
cmake --build build --target install --parallel [num CPU cores]
```

> ℹ️ Make sure to check out our [template C++ project](https://github.com/luxonis/depthai-core-example).

### Verifying installation
To verify the installation works as expected, you can test if the integration project compiles and runs.
This is done by running the following command:

```
cmake -S tests/integration . -B build_integration -D'CMAKE_PREFIX_PATH=[path/to/install/dir]'
cmake --build build_integration --target test --parallel [num CPU cores]
```

### Prebuilt library on Windows
Under releases you may find prebuilt library for Windows, for use in either integration method. See [Releases](https://github.com/luxonis/depthai-core/releases)

### Using find_package for integration

> ℹ️ Due to a non-trivial dependency tree, the integration with `add_subdirectory` is not supported. Use `find_package` instead.

First install the library as described in [Installation and Integration](#installation-and-integration) section.

Then in your CMake project, add the following lines to your `CMakeLists.txt` file:

```cmake

# Add `find_package` and `target_link_libraries` to your project

find_package(depthai CONFIG REQUIRED)
...
target_link_libraries([my-app] PRIVATE depthai::core)
```

And point CMake to your install directory:
```
-D'CMAKE_PREFIX_PATH=[path/to/install/dir]'
```


If library was installed to default search path like `/usr/local` on Linux, specifying `CMAKE_PREFIX_PATH` isn't necessary as CMake will find it automatically.


### Vcpkg integration
For VCPKG integration, check out the example [here](https://github.com/luxonis/depthai_vcpkg_example).
Note that the VCPKG integration is using a custom branch of DepthAI and we plan to integrate the support for it into the main branch in the future and add it to the official VCPKG repository.


### Android
Android is not yet supported on the v3.x.y version of DepthAI. You can still use the v2.x.y version of DepthAI for RVC2 devices or open an issue on this repository to request Android support for v3.x.y.

<!-- Steps:

 - Install Android NDK (for example via Android Studio).
 - Set the NDK path:
```
export ANDROID_HOME=$HOME/.local/lib/Android
export PATH=$PATH:$ANDROID_HOME/emulator:$ANDROID_HOME/platform-tools
export NDK=$ANDROID_HOME/ndk/23.1.7779620/ # Check version
```
 - Ensure a recent version of cmake (apt version is outdated, install snap install cmake --classic)
 - Run cmake, set your ABI and Platform as needed:

```
cmake -S. -Bbuild -DCMAKE_TOOLCHAIN_FILE=$NDK/build/cmake/android.toolchain.cmake -DANDROID_ABI=armeabi-v7a -DANDROID_PLATFORM=android-25
cmake --build build
``` -->



<!-- ### Non-CMake integration (Visual Studio, Xcode, CodeBlocks, ...)

To integrate into a different build system than CMake, prefered way is compiling as dynamic library and setting correct build options.
1. First build as dynamic library: [Building Dynamic library](#dynamic-library)
2. Then install: [Installing](#installing)

In your non-CMake project (new Visual Studio project, ...)
1. Set needed library directories:
    - `build/install/lib` (for linking to either depthai-core or depthai-opencv)
    - `build/install/bin` (for .dll's)
2. And include directories
    - `build/install/include` (library headers)
    - `build/install/include/depthai-shared/3rdparty` (shared 3rdparty headers)
    - `build/install/lib/cmake/depthai/dependencies/include` (dependency headers)

> ℹ️ Threading library might need to be linked to explicitly.

> ℹ️ Check `build/depthai-core-integration.txt` or `build/depthai-opencv-integration.txt` for up to date define options.
The generated integration file also specifies include paths without requiring installation. -->

## CMake options
Many features of the library can be disabled or enabled using CMake options.
One common option when building the library tests and examples is `DEPTHAI_VCPKG_INTERNAL_ONLY=OFF` which installs a predictable version of OpenCV, PCL and other optional dependencies we also use on the libraries interface.

> ℹ️ When `DEPTHAI_VCPKG_INTERNAL_ONLY=OFF` is used, the library cannot be installed (apart from being installed in the scope of the vcpkg package manager).

For a full list of options, see `cmake/depthaiOptions.cmake` file.

### Minimal CMake preset

The `minimal` preset in `CMakePresets.json` configures a lean build that disables optional components such as libusb interaction, AprilTag, Protobuf, CURL, OpenCV, tests/examples/docs, remote connection support, and embedded firmware resources. xtensor support remains enabled because it is required by the detection parser APIs.

To configure and build this minimal setup:

```
cmake --preset minimal
cmake --build --preset minimal --parallel
```

Dependencies are resolved through vcpkg automatically during configuration.

## Environment variables

The following environment variables can be set to alter default behavior of the library without having to recompile

| Environment variable  | Description   |
|--------------|-----------|
| DEPTHAI_LEVEL | Sets logging verbosity, 'trace', 'debug', 'info', 'warn', 'error' and 'off' |
| XLINK_LEVEL | Sets logging verbosity of XLink library, 'debug'. 'info', 'warn', 'error', 'fatal' and 'off' |
| DEPTHAI_INSTALL_SIGNAL_HANDLER | Set to 0 to disable installing Backward signal handler for stack trace printing |
| DEPTHAI_WATCHDOG | Sets device watchdog timeout. Useful for debugging (`DEPTHAI_WATCHDOG=0`), to prevent device reset while the process is paused. |
| DEPTHAI_WATCHDOG_INITIAL_DELAY | Specifies delay after which the device watchdog starts. |
| DEPTHAI_SEARCH_TIMEOUT | Specifies timeout in milliseconds for device searching in blocking functions. |
| DEPTHAI_CONNECT_TIMEOUT | Specifies timeout in milliseconds for establishing a connection to a given device. |
| DEPTHAI_BOOTUP_TIMEOUT | Specifies timeout in milliseconds for waiting the device to boot after sending the binary. |
| DEPTHAI_RECONNECT_TIMEOUT | Specifies timeout in milliseconds for reconnecting to a device after a connection loss. If set to 0, reconnect is disabled. |
| DEPTHAI_PROTOCOL | Restricts default search to the specified protocol. Options: `any`, `usb`, `tcpip`, `tcpshd`. |
| DEPTHAI_PLATFORM | Restricts default search to the specified platform. Options: `any`, `rvc2`, `rvc3`, `rvc4`. |
| DEPTHAI_DEVICE_MXID_LIST | Restricts default search to the specified MXIDs. Accepts comma separated list of MXIDs. Lists filter results in an "AND" manner and not "OR" |
| DEPTHAI_DEVICE_ID_LIST | Alias to MXID list. Lists filter results in an "AND" manner and not "OR" |
| DEPTHAI_DEVICE_NAME_LIST | Restricts default search to the specified NAMEs. Accepts comma separated list of NAMEs. Lists filter results in an "AND" manner and not "OR". It also looks for NAMEs outside of the host's subnet in case of tcpip. |
| DEPTHAI_DEVICE_BINARY | Overrides device Firmware binary. Mostly for internal debugging purposes. |
| DEPTHAI_DEVICE_RVC4_FWP | Overrides device RVC4 Firmware binary. Mostly for internal debugging purposes. |
| DEPTHAI_BOOTLOADER_BINARY_USB | Overrides device USB Bootloader binary. Mostly for internal debugging purposes. |
| DEPTHAI_BOOTLOADER_BINARY_ETH | Overrides device Network Bootloader binary. Mostly for internal debugging purposes. |
| DEPTHAI_ALLOW_FACTORY_FLASHING | Internal use only |
| DEPTHAI_LIBUSB_ANDROID_JAVAVM | JavaVM pointer that is passed to libusb for rootless Android interaction with devices. Interpreted as decimal value of uintptr_t |
| DEPTHAI_CRASHDUMP | Directory in which to save the crash dump. |
| DEPTHAI_CRASHDUMP_TIMEOUT | Specifies the duration in milliseconds to wait for device reboot when obtaining a crash dump. Crash dump retrieval disabled if 0. |
| DEPTHAI_ENABLE_ANALYTICS_COLLECTION | Enables automatic analytics collection (pipeline schemas) used to improve the library |
| DEPTHAI_DISABLE_CRASHDUMP_COLLECTION | Disables automatic crash dump collection used to improve the library |
| DEPTHAI_HUB_EVENTS_BASE_URL | URL for events of the Luxonis Hub |
| DEPTHAI_HUB_API_KEY | API key for the Luxonis Hub |
| DEPTHAI_ZOO_INTERNET_CHECK | (Default) 1 - perform internet check, if available, download the newest model version 0 - skip internet check and use cached model |
| DEPTHAI_ZOO_INTERNET_CHECK_TIMEOUT | (Default) 1000 - timeout in milliseconds for the internet check |
| DEPTHAI_ZOO_CACHE_PATH | (Default) .depthai_cached_models - Folder where cached zoo models are stored |
| DEPTHAI_ZOO_MODELS_PATH | (Default) depthai_models - Folder where zoo model description files are stored |
| DEPTHAI_RECORD | Enables holistic record to the specified directory. |
| DEPTHAI_REPLAY | Replays holistic replay from the specified file or directory. |
| DEPTHAI_PROFILING | Enables runtime profiling of data transfer between the host and connected devices. Set to 1 to enable. Requires DEPTHAI_LEVEL=debug or lower to print. |

## Running tests

To run the tests build the library with the following options
```
cmake -S. -Bbuild -D'DEPTHAI_TEST_EXAMPLES=ON' -D'DEPTHAI_BUILD_TESTS=ON' -D'DEPTHAI_BUILD_EXAMPLES=ON'
cmake --build build
```

Then navigate to `build` folder and run `ctest` with specified labels that denote device type to test on.
Currently available labels:
 - usb
 - poe
 - rvc2
 - rvc4

```
cd build
# Run tests on RVC2 devices
ctest -L rvc2
# Run tests on RVC4 devices
ctest -L rvc4
```

## Style check

The library uses clang format to enforce a certain coding style.
If a style check is failing, run the `clangformat` target, check the output and push changes.

To use this target clang format must be installed, preferably clang-format-18
```
sudo apt install clang-format-18
```

And to apply formatting
```
cmake --build build --target clangformat
```

## Documentation generation

Doxygen is used to generate documentation. Follow [doxygen download](https://www.doxygen.nl/download.html#srcbin) and install the required binaries for your platform.

After that specify CMake define `-D'DEPTHAI_BUILD_DOCS=ON'` and build the target `doxygen`

## Debugging tips

Debugging can be done using **Visual Studio Code** and either **GDB** or **LLDB** (extension 'CodeLLDB').
LLDB in some cases was much faster to step with and resolved more `incomplete_type` variables than GDB. Your mileage may vary though.


## Troubleshooting

### Build fails with missing OpenCV dependency

If your build process happen to fail due to OpenCV library not being found, but you have the OpenCV installed, please
run build with additional `-D'OpenCV_DIR=...` flag (replacing default Ubuntu path `/usr/lib/x86_64-linux-gnu/cmake/opencv4` with yours)

```
cmake -S. -Bbuild -D'OpenCV_DIR=/usr/lib/x86_64-linux-gnu/cmake/opencv4'
```

Now the build process should correctly discover your OpenCV installation

### Build fails due to out of memory killer
If your build process is killed by the out of memory killer, you can try to reduce the number of parallel jobs used during the build process.

The error usually looks something like this:
```
c++: fatal error: Killed signal terminated program cc1plus
```

You can do this by passing the `--parallel` flag with a lower number of jobs to the `cmake --build` command, for example:
```
cmake --build build --parallel 2
```

