# RVC4 device firmware (`device/rvc4`)

Device-side firmware of the DepthAI platform for RVC4 devices: Luxonis OS on the Qualcomm QCS8550. The firmware
runs on the device, executes the pipeline that a `depthai-core` application builds on the host, and streams the
results back to the host over XLink (TCP/IP or USB).

The firmware is part of the `depthai-core` repository. It compiles the node definitions, message types and
pipeline logic of the same checkout (two directories up), so the host library and the firmware of one commit
always agree on the pipeline schema.

## Architecture in ten lines

1. Luxonis OS runs the `depthai_gate` service. It unpacks the firmware package (fwp), runs the hooks in
   `scripts/custom_scripts.py` (`on_start`, `on_stop`, ...) and spawns the firmware binary `depthai-device-rvc4`.
2. A host application built with `depthai-core` discovers the device and connects to the firmware over XLink.
3. The firmware serves an RPC interface on the main XLink channel (`src/XLinkRpc.cpp`, nanorpc with msgpack).
4. The host serialises its `Pipeline` to a `PipelineSchema` and sends it with the `setPipelineSchema` RPC call.
5. `src/pipeline/PipelineBuilder.cpp` creates one device node per schema node and links their inputs and outputs.
6. Device nodes (`src/pipeline/node/`) subclass the `depthai-core` node classes (`dai::node::*`): properties and
   messages come from `depthai-core`, the device implementation (cameras, GPU, DSP, NPU) is in this directory.
7. Cameras are driven through the Qualcomm camera HAL and GStreamer (`src/camera/`); frames live in DMA buffers.
8. Neural networks run on the Hexagon NPU through SNPE; image alignment and stereo depth run on the Hexagon DSP
   (`src/hexagon/`).
9. Output messages go back to the host over XLink streams (`XLinkOut` nodes); logs, telemetry and the watchdog
   share the same link.
10. When the pipeline stops, the hooks restore the device state and `depthai_gate` takes over again.

## Directory layout

```
CMakeLists.txt          Firmware build: depthai-device-core library, depthai-device-rvc4 executable, fwp packaging.
                        It is a separate CMake project that adds depthai-core (../..) as a subdirectory.
src/                    Firmware sources
  main.cpp              Entry point: XLink server, RPC handlers, pipeline lifecycle
  pipeline/             PipelineBuilder, frame pools, node implementations (pipeline/node/)
  camera/               Camera HAL integration, sensors, ISP routing
  driver/               Low-level device drivers (SPI)
  hexagon/              Hexagon DSP offload (ImageAlignDsp, StereoDepthDsp)
  utilities/            Discovery, telemetry, GPIO, memory helpers
include/                Public headers of the firmware library
scripts/                Device-side hooks, packaged into the fwp and executed by depthai_gate
external/cpp-subprocess Submodule (process helper)
cmake/, toolchains/, triplets/, vcpkg.json   Build system: OE toolchain file, vcpkg triplets and manifest
vcpkg/                  vcpkg checkout of the pinned commit; the configure step fetches it (not tracked)
docker/build.sh         Build entry point that runs inside the SDK container
utility_scripts/        Developer helpers: build, deploy, debug, core dumps, license bundle and SBOM generation
.devcontainer/, .vscode/, vscode_development/   VS Code dev-container setup
```

## Building

The firmware is cross-compiled inside a Docker image that contains the Luxonis OS RVC4 SDK (OpenEmbedded
toolchain, sysroot with the Qualcomm libraries, Hexagon SDK, gdb). Set `DEPTHAI_DEVICE_SDK_IMAGE` to the image.

At this time the build needs two things that are not public:

* the Luxonis OS SDK image, and
* the sources of the private module `depthai-device-rvc4-vendor` (licensee-only Qualcomm and ST code, see
  [What is not in this repository](#what-is-not-in-this-repository)).

Both are available to Luxonis licensees. A public build container and a prebuilt archive of the private module
are in preparation.

The private module is the private repository `depthai-device-vendor-fw`. The build scripts look for a checkout of it
next to the `depthai-core` checkout (`../depthai-device-vendor-fw`), or at the path in `DEPTHAI_DEVICE_RVC4_VENDOR_SOURCE_DIR`. They mount it at
`/depthai-device-rvc4-vendor` in the container and pass that path to CMake (`DEPTHAI_DEVICE_RVC4_VENDOR_SOURCE_DIR`).

```bash
git clone --recursive https://github.com/luxonis/depthai-core.git
cd depthai-core/device/rvc4
export DEPTHAI_DEVICE_SDK_IMAGE=<Luxonis OS RVC4 SDK image>
./create_deployment_artifacts.sh rvc4          # or rvc4-asan-ubsan, rvc4-tsan
```

`create_deployment_artifacts.sh` mounts the `depthai-core` checkout at `/work` and runs `docker/build.sh` in
`/work/device/rvc4`. `DEPTHAI_BUILD_JOBS` sets the number of parallel compile jobs. The first configure step
fetches vcpkg (pinned commit) into `device/rvc4/vcpkg`.

The output is the firmware package

```
device/rvc4/build_docker_arm64_rvc4/RelWithDebInfo/depthai-device-rvc4-fwp.tar.xz
```

together with its license bundle (`depthai-device-rvc4-fwp-LICENSE`) and SBOMs (`.spdx.json`, `.cdx.json`).

The DSP offload (`src/hexagon/`) is built from source with the Qualcomm Hexagon SDK 5.5.0.1; CMake stops when
`HEXAGON_SDK_ROOT` does not point at a Hexagon SDK. The Luxonis OS SDK image contains the SDK. With an image
without it, set `HEXAGON_SDK_DIR` to a Hexagon SDK on the host: the scripts mount it at `/opt/hexagon-sdk` and set
`HEXAGON_SDK_ROOT`.

For iteration, do not rebuild from scratch. Start a shell in the SDK container with `./start_dev_shell.sh rvc4`
and run an incremental build:

```bash
cmake --build build_docker_arm64_rvc4 --target depthai-device-rvc4 --config RelWithDebInfo --parallel 10
```

`utility_scripts/build.sh` runs the same command from the host in the running container. For VS Code with
code completion and one-click debugging, see [vscode_development](vscode_development/vscode_development.md).

## Running a custom build with depthai-core

A host build of `depthai-core` contains the firmware package that it pins. To run your own build instead, point
`depthai-core` at your package at run time:

```bash
export DEPTHAI_DEVICE_RVC4_FWP=/path/to/depthai-core/device/rvc4/build_docker_arm64_rvc4/RelWithDebInfo/depthai-device-rvc4-fwp.tar.xz
```

Then run any `depthai-core` example or your own application. `depthai-core` logs that the firmware is overridden,
uploads your package to the device and starts it. Build the host library and the firmware from the same commit, so
that the pipeline schema matches.

## Testing

The on-device tests of `depthai-core` (`tests/` at the repository root, label `rvc4`) run against a firmware
package that you select with `DEPTHAI_DEVICE_RVC4_FWP`.

## Debugging

* Stop the gate on the device so that it releases the discovery port and does not restart the firmware:
  `systemctl stop depthai_gate` (and `systemctl start depthai_gate` when you are done).
* Run the firmware under `gdbserver` on the device and connect with the gdb of the SDK container, or with
  `gdb-multiarch` on the host. `utility_scripts/` has scripts for the build, deploy and debug steps.
* The build strips the binary and keeps the symbols in `depthai-device-rvc4.debug` next to it; load that file in
  gdb (`-symbols`) and set the sysroot to the SDK sysroot so shared libraries resolve.

See [debug.md](debug.md) and [utility_scripts/README.md](utility_scripts/README.md) for the details, including how
to attach to a stuck production firmware and how to parse core dumps.

## Licensing

* The source code in this directory is licensed under the MIT License of the `depthai-core` repository, see the
  `LICENSE` file in the repository root. [NOTICE](NOTICE) lists the third-party attributions.
* The binary firmware package (`depthai-device-rvc4-fwp.tar.xz`) bundles proprietary components (the ST VD55H1
  IPP library under ST SLA0106 and Qualcomm-derived code from the private module) and is distributed under the
  Luxonis Firmware Package License, see [LICENSE.fwp](LICENSE.fwp). The build writes the full license bundle of
  the package, with every third-party notice, to `depthai-device-rvc4-fwp-LICENSE` next to the package, and the
  SBOMs in SPDX 2.3 and CycloneDX 1.6 format.

## What is not in this repository

The private module `depthai-device-rvc4-vendor` contains the code that depends on licensee-only Qualcomm and ST
material and cannot be published:

* EVA (Qualcomm Engine for Visual Analytics) integration
* SEE sensor framework integration (IMU and other sensors)
* CHI-CDK camera HAL pieces
* ST ToF support: the VD55H1 IPP library and the depthmapwrapper

The public code uses these parts through Luxonis-owned interface headers (for example
`src/utilities/EvaDataMemory.hpp`, `src/pipeline/node/Warp/rvc4/WarpDispatcherRvc4.hpp`,
`src/pipeline/node/IMU/rvc4/SensorDirectChannel.hpp`). At this time the firmware cannot be built without the
sources of the module.
