# DepthAI Docker image

The [`luxonis/depthai-library` image](https://hub.docker.com/r/luxonis/depthai-library)
contains the installed C++ SDK, headers, CMake package, Python bindings, and build
tools. The [publishing workflow](../.github/workflows/containers.yml) builds Linux
amd64 and arm64 images from [ci/Dockerfile](../ci/Dockerfile).

Release tags use the matching Git tag, such as `v3.x.y`; `latest` is updated by
release-tag builds. Non-release builds use the full source commit as their tag.
Choose an existing version tag from Docker Hub for repeatable deployments.

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

Combine the networking and USB options when using both transports. The host user
must have permission to access the cameras; see the
[Linux setup guide](https://docs.luxonis.com/software-v3/depthai/getting-started/).
These device-access commands target Linux Docker Engine. Docker Desktop on macOS
or Windows runs a Linux VM and needs its own USB/network forwarding setup.

## Use a project checkout

Mount your project separately from the installed SDK. From your project root:

```sh
docker run --rm -it --network=host \
  --mount type=bind,source="$PWD",target=/workspace \
  --workdir /workspace \
  luxonis/depthai-library:<tag> bash
```

Inside the container, a CMake project can use `find_package(depthai CONFIG REQUIRED)`
and link `depthai::core`; Python applications can import `depthai` directly.
Add the USB options above if the project uses USB cameras.

## Build the SDK image from source

The repository Dockerfile builds and installs the SDK and repairs the Python
wheel. From the desired checkout:

```sh
git submodule update --init --recursive
docker build -f ci/Dockerfile -t depthai-library:local .
```

This performs a complete native build, so it takes longer than starting a
published image. Files created in a bind mount keep the container user's
ownership; choose a suitable container user when writing to your host checkout.
