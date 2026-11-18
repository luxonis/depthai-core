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

The firmware is cross-compiled in a Docker image on an x86-64 host. These images can build it:

* the public build images `ghcr.io/luxonis/depthai-device-sdk:os-<OS version>-<sysroot>` (for the release firmware)
  and the same tag with `-dev` (the same plus the sanitizer libraries): Ubuntu 22.04, the cross compiler of Ubuntu (GCC 11.4)
  and the public sysroot of Luxonis OS. Anyone can use them, see [Public build container](#public-build-container).
* the Luxonis OS SDK image: the OpenEmbedded toolchain, the full sysroot and the Hexagon SDK (Luxonis licensees).

CMake selects the toolchain with `DEPTHAI_DEVICE_RVC4_TOOLCHAIN`: `auto` (default) takes `oe` when the OpenEmbedded
SDK environment is sourced (the Luxonis OS SDK image), else `gnu` (`toolchains/rvc4-gnu.cmake`). Both use the same
compile and link flags.

The firmware links the vendor module `depthai-device-rvc4-vendor` (licensee-only Qualcomm and ST code of the private
repository `depthai-device-vendor-fw`, see [What is not in this repository](#what-is-not-in-this-repository)) as a prebuilt
archive, `depthai-device-rvc4-vendor-<version>[-asan-ubsan|-tsan].tar.xz`:

* By default the build downloads the archive of `DEPTHAI_DEVICE_RVC4_VENDOR_VERSION` in
  `cmake/Depthai/DepthaiDeviceRVC4Config.cmake` of depthai-core (`cmake/DepthaiDeviceRvc4VendorDownloader.cmake`) from
  `artifacts.luxonis.com`, checks its sha256 and keeps it in the build folder.
* `DEPTHAI_DEVICE_RVC4_VENDOR=<path of an archive>` takes a local archive in place of the download, as
  `DEPTHAI_DEVICE_RVC4_FWP` takes a local firmware package for depthai-core. The scripts mount it into the container.

The CI of `depthai-device-vendor-fw` builds the module from its sources and publishes the archive: with `DEPTHAI_DEVICE_RVC4_VENDOR_SOURCE_DIR` (Luxonis
licensees) the scripts mount the sources at `/depthai-device-rvc4-vendor` and pass them to CMake as
`DEPTHAI_DEVICE_RVC4_VENDOR_SOURCE_DIR`.

A prebuilt archive was compiled against the public headers of the depthai-core commit of its build. CMake compares every header and
interface define that it used with this tree, and stops with the list of differences when one differs: a changed
header can change the binary interface. The configure step checks the headers; the build checks them again, and the
defines, before the code that uses the archive compiles. With a prebuilt archive the ToF sensor pipeline is a stub
(the ToF node reports an error).

```bash
git clone --recursive https://github.com/luxonis/depthai-core.git
cd depthai-core/device/rvc4
DEPTHAI_ACCEPT_QUALCOMM_LICENSE=yes ./create_deployment_artifacts.sh rvc4          # or rvc4-asan-ubsan, rvc4-tsan
```

`create_deployment_artifacts.sh` mounts the `depthai-core` checkout at `/work` and runs `docker/build.sh` in
`/work/device/rvc4`. Without `DEPTHAI_DEVICE_SDK_IMAGE` it pulls the public build image that the target needs: the
release image for `rvc4`, the dev image for the sanitizer targets. `DEPTHAI_ACCEPT_QUALCOMM_LICENSE=yes` is necessary
only for the first build with a public image: it lets the script download the parts of the Qualcomm AI Runtime SDK
that the build uses, see
[Public build container](#public-build-container). `DEPTHAI_BUILD_JOBS` sets the number of parallel compile jobs. The
first configure step fetches vcpkg (pinned commit) into `device/rvc4/vcpkg`.

A build of the target `rvc4` compiles 530 source files: depthai-core, its Python bindings (the Script node uses
them) and the firmware sources. Without help this takes about 10 minutes with 10 compile jobs. Three things make it
short:

* **Published build cache.** For each commit of the main branches and for each release, a CI job publishes the object
  files of depthai-core, of its Python bindings and of most firmware sources (a compiler cache, about 60 MB), and the
  vcpkg dependencies (binary archives, about 45 MB). The build downloads them for the commit of your checkout, or for
  the nearest of its 10 parent commits that has them, and compiles only the rest. A source that you changed is
  compiled: an object file of the cache is used only when the source and all its headers are the same. The firmware
  sources that include headers of the Qualcomm AI Runtime SDK or of FastCV are not in the published cache; your build
  compiles them. With the published cache of the checkout commit, a first build takes about 70 s.
  `DEPTHAI_DEVICE_BUILD_CACHE=0` turns the download off.
* **Caches on the host** that all checkouts share, in `~/.cache/depthai-device-sdk` (or `DEPTHAI_DEVICE_SDK_CACHE`):
  `ccache`, the compiler cache (`CCACHE_MAXSIZE` sets its size limit, default 10G), `vcpkg`, the binary cache of the
  vcpkg dependencies, and `build-cache`, the downloads above. A new clone or a new git worktree takes the object
  files of earlier builds from them. A CI job (environment variable `CI`) keeps its caches in
  `device/rvc4/build_docker_arm64` and does not use the published build cache.
* **No debug data.** The target `rvc4` with the prebuilt private module compiles without debug data by default. The
  firmware package is the same, because it has the stripped binary: its machine code and data are byte-equal. Set
  `DEPTHAI_DEVICE_DEBUG_INFO=1` when you use a debugger or when you need the file and line of an address in a crash
  report; the build is then slower (the link step takes 25 s for 9 s, the build folder is 5.9 GB for 0.5 GB) and it
  does not use the published build cache. The sanitizer targets and a build with the sources of the private module
  have debug data by default.

A build with no change does no work, and a build after a change compiles only the changed sources and links the
firmware again. The build time that the firmware reports is the time of the commit (`SOURCE_DATE_EPOCH`), so that two
builds of one commit give the same binary. A compile job can use 2 GB to 5 GB of memory, so the default of
`DEPTHAI_BUILD_JOBS` is half of the CPU cores.

The output is the firmware package

```
device/rvc4/build_docker_arm64_rvc4/RelWithDebInfo/depthai-device-rvc4-fwp.tar.xz
```

together with its license bundle (`depthai-device-rvc4-fwp-LICENSE`) and SBOMs (`.spdx.json`, `.cdx.json`).

The DSP offload (`src/hexagon/`) is built from source when `HEXAGON_SDK_ROOT` points at the Qualcomm Hexagon SDK
5.5.0.1; the Luxonis OS SDK image contains it. With another image, set `HEXAGON_SDK_DIR` to a Hexagon SDK on the
host: the scripts mount it at `/opt/hexagon-sdk` and set `HEXAGON_SDK_ROOT`. Without the SDK the DSP offload comes
from the `dsp/` folder of the prebuilt archive, and CMake checks that `src/hexagon/` is the same as in its build.

### Public build container

Luxonis builds and publishes two images from `docker/Dockerfile.sdk`. You pull them; you do not build them:

| Image | For | Content |
|---|---|---|
| `ghcr.io/luxonis/depthai-device-sdk:os-<OS version>-<sysroot>` | the release firmware (target `rvc4`) | Ubuntu 22.04, the build tools, the cross compiler of Ubuntu (`g++-11-aarch64-linux-gnu`, GCC 11.4 and binutils 2.38, the versions of the Luxonis OS SDK) without its own C library, the public sysroot of Luxonis OS `<OS version>`, the FastCV headers |
| `ghcr.io/luxonis/depthai-device-sdk:os-<OS version>-<sysroot>-dev` | the targets `rvc4-tsan` and `rvc4-asan-ubsan`, and `start_dev_shell.sh` | the same plus the sanitizer libraries of the compiler |

`<sysroot>` is the start of the sha256 of the public sysroot. A tag with it never moves to another image. The tags
`os-<OS version>` and `os-<OS version>-dev` are the newest images for that OS version. depthai-core pins the image
in `cmake/Depthai/DepthaiDeviceRVC4Config.cmake` (`DEPTHAI_DEVICE_RVC4_SDK_IMAGE`), and the build scripts use the
pinned image when `DEPTHAI_DEVICE_SDK_IMAGE` is not set. The OS version of the image is the one of the prebuilt
vendor module (`DEPTHAI_DEVICE_RVC4_VENDOR_VERSION`): the build compares them.

The images have no Qualcomm software that Luxonis may not redistribute: the sysroot has link stubs of the Qualcomm
libraries (same names and symbols, no code; the device has the real libraries). Two Qualcomm SDKs give headers to
the build:

* **FastCV**: its headers are in the images. Qualcomm Innovation Center publishes them under the BSD-3-Clause-Clear
  license (the license and notice files are in `/opt/rvc4-sysroot/usr/share/licenses/fastcv` of the images).
* **QAIRT** (Qualcomm AI Runtime SDK, version 2.32.6.250402): not in the images, because Luxonis may not
  redistribute it. The build scripts keep the parts that the build uses (the SNPE headers and `libSNPE.so` of the
  device OS target, 22 MB) in a cache on the host, `~/.cache/depthai-device-sdk` (or `DEPTHAI_DEVICE_SDK_CACHE`), and
  mount them into the sysroot of the container. The first build fills the cache in one of two ways:
  * `DEPTHAI_ACCEPT_QUALCOMM_LICENSE=yes`: the script downloads these parts from Qualcomm (about 10 MB of the 1.1 GB
    SDK archive, one time) and checks their checksum. With this value you accept the license terms of Qualcomm for
    the SDK (`LICENSE.pdf` in the archive; a copy stays in the cache). These terms also apply to the firmware that
    you build with it.
  * `DEPTHAI_DEVICE_QAIRT_ARCHIVE=/path/to/v2.32.6.250402.zip`: an archive that you downloaded from Qualcomm.

  An image that has QAIRT in its sysroot (the Luxonis OS SDK image) needs neither.

`install-hexagon-sdk.sh` in the images puts the Qualcomm Hexagon SDK into an image of your own, to build the DSP
offload from source. `HEXAGON_SDK_DIR` (see above) does the same without a new image.

The public sysroot is made in the CI of `depthai-device-vendor-fw`, from the Luxonis OS SDK image. When the sysroot
changes, that CI builds both images with `docker/sdk-images.sh`, checks them and pushes them:

```bash
docker/sdk-images.sh build rvc4-public-sysroot-<version>.tar.xz   # build, tag and check both images
docker/sdk-images.sh push rvc4-public-sysroot-<version>.tar.xz    # push the four tags
docker/sdk-images.sh check                                        # check the pinned images
```

The GitHub workflow `containers_device_rvc4_sdk.yml` checks the pinned images when `docker/` or the pin changes. It
does not build them.

For iteration, do not rebuild from scratch. Start a shell in the SDK container with `./start_dev_shell.sh rvc4`
and run an incremental build:

```bash
cmake --build build_docker_arm64_rvc4 --target depthai-device-rvc4 --config RelWithDebInfo --parallel 10
```

`utility_scripts/build.sh` runs the same command from the host in the running container. For VS Code with
code completion and one-click debugging, see [vscode_development](vscode_development/vscode_development.md).

## Running a custom build with depthai-core

A host build of `depthai-core` builds the firmware from this directory and embeds it, when
`cmake/Depthai/DepthaiDeviceRVC4Config.cmake` sets no `DEPTHAI_DEVICE_RVC4_VERSION` (the default). Each build of the
library runs `create_deployment_artifacts.sh` for the target that fits the host build (`rvc4`, or `rvc4-tsan` /
`rvc4-asan-ubsan` for a sanitizer build), and embeds the package as version `<version>+local.<commit>`. With
`-D DEPTHAI_DEVICE_RVC4_VERSION=<version>` the host build downloads the package of that version instead.

To run another package without a new host build, point `depthai-core` at it at run time:

```bash
export DEPTHAI_DEVICE_RVC4_FWP=/path/to/depthai-core/device/rvc4/build_docker_arm64_rvc4/RelWithDebInfo/depthai-device-rvc4-fwp.tar.xz
```

Then run any `depthai-core` example or your own application. `depthai-core` logs that the firmware is overridden,
uploads your package to the device and starts it. Build the host library and the firmware from the same commit, so
that the pipeline schema matches.

To put a package that you have into the host library, configure the host build with the path:

```bash
cmake -S . -B build -D DEPTHAI_DEVICE_RVC4_FWP_PATH=$PWD/device/rvc4/build_docker_arm64_rvc4/RelWithDebInfo/depthai-device-rvc4-fwp.tar.xz
```

The library (and a Python wheel that you build from it) then contains your firmware and needs no environment
variable. The version of the embedded firmware is `<version>+local.<hash of the package>`. A new package changes
the version and the host library is compiled again, so use the environment variable while you iterate.

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
