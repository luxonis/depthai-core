# DepthAI Docker image

The [`luxonis/depthai-library` image](https://hub.docker.com/r/luxonis/depthai-library)
contains the installed C++ SDK, headers, CMake package, Python bindings, and build
tools. The [publishing workflow](../.github/workflows/containers.yml) builds Linux
amd64 and arm64 images from [ci/Dockerfile](../ci/Dockerfile).

Release tags use the matching Git tag, such as `v3.x.y`; `latest` is updated by
release-tag builds. Non-release builds use the full source commit as their tag.
Only commits published by that workflow have image tags. Choose a published v3
tag from the [Docker Hub tags page](https://hub.docker.com/r/luxonis/depthai-library/tags)
for repeatable deployments; Docker selects the host architecture automatically.

## Check the installed SDK

Replace `<tag>` with a published tag:

```sh
docker run --rm luxonis/depthai-library:<tag> \
  python3 -c "import depthai as dai; print(dai.__version__); print(dai.__file__)"
```

The image build also compiles a small C++ program using
`find_package(depthai CONFIG REQUIRED)` and `depthai::core`.

## Connect to cameras on a Linux host

USB discovery needs access to the host's USB device nodes. Share the USB bus and
allow its device major so devices can disappear and re-enumerate during boot:

```sh
docker run --rm -it \
  --mount type=bind,source=/dev/bus/usb,target=/dev/bus/usb \
  --device-cgroup-rule='c 189:* rmw' \
  luxonis/depthai-library:<tag> \
  python3 -c "import depthai as dai; print(dai.Device.getAllAvailableDevices())"
```

For Ethernet cameras, Linux host networking lets discovery broadcasts reach the
camera's network:

```sh
docker run --rm -it --network=host luxonis/depthai-library:<tag> \
  python3 -c "import depthai as dai; print(dai.Device.getAllAvailableDevices())"
```

Combine the networking and USB options when using both transports. For USB access
as a non-root container user, install the udev rules on the **host**; see the
[Linux USB setup guide](https://docs.luxonis.com/hardware/platform/deploy/usb-deployment-guide#linux-udev-rules).
These device-access commands target Linux Docker Engine. Docker Desktop on macOS
or Windows runs a Linux VM and needs its own USB/network forwarding setup.

## Build an application with the installed SDK

Mount your application separately from the installed SDK. From your project root
on Linux, run:

```sh
docker run --rm -it --network=host \
  --user "$(id -u):$(id -g)" \
  --mount type=bind,source="$PWD",target=/workspace \
  --workdir /workspace \
  luxonis/depthai-library:<tag> bash
```

The user option keeps generated files owned by your host user. On SELinux hosts
such as Fedora, replace the project mount with `--volume "$PWD:/workspace:Z"`
to label the checkout for container access.

Inside the container, a CMake application can use `find_package(depthai CONFIG REQUIRED)`
and link `depthai::core`, as shown in the [integration guide](../README.md#using-find_package-for-integration).
The SDK is installed under `/usr/local`, which CMake searches automatically.
Configure and build your application in a separate directory from host builds:

```sh
cmake -S . -B build-docker -DCMAKE_BUILD_TYPE=Release
cmake --build build-docker --parallel 2
```

Python applications can import `depthai` directly. Install any additional
application dependencies separately. The image uses `opencv-python-headless`;
examples that open GUI windows need a GUI-enabled OpenCV and display setup.
Add the USB options above when starting the container if the application uses
USB cameras.

## Build the SDK image from source

The repository Dockerfile builds and installs both the C++ SDK and Python bindings
from your checkout, including any local changes, and repairs the Python wheel.
From the depthai-core repository root:

```sh
git submodule update --init --recursive
rm -rf /tmp/rvc4-fwp
ci/device-rvc4-fwp-ak.sh download "$(ci/device-rvc4-fwp-ak.sh version .)" rvc4 /tmp/rvc4-fwp &&
docker build -f ci/Dockerfile -t depthai-library:local \
  --build-context rvc4-fwp=/tmp/rvc4-fwp \
  --build-arg DEPTHAI_DEVICE_RVC4_FWP_PATH=/rvc4-fwp/depthai-device-rvc4-fwp.tar.xz .
```

CI puts a firmware package on Artifact Keeper for each push to `develop`, `main`,
a `release*` branch or a `v*` tag (`rvc4_fwp_publish.yml`). For another commit,
`download` exits with 2. Then use the package of a commit with the same
`device/rvc4` and shared headers, for example the merge base
(`<first line of device/rvc4/version.txt>+<commit>`), or build one from
`device/rvc4` (see `device/rvc4/README.md`) and give its folder. The firmware
package does not contain your local changes in `device/rvc4`.

This performs a complete native build, so it takes longer than starting a
published image and needs network access to download dependencies and firmware.
Use `depthai-library:local` in place of `luxonis/depthai-library:<tag>` in the
commands above to use the resulting image. The final image contains installed
artifacts; the SDK source and its build directories stay in the builder stage.
