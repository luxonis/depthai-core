# PointCloud Examples (C++)

## PointCloudShowcase.cpp

Demonstrates all major capabilities of the `PointCloud` node in a single pipeline.
A shared `Camera` + `StereoDepth` pair fans its depth output into four `PointCloud`
nodes, each configured differently:

| # | Feature | Key API |
|---|---------|---------|
| 1 | Length unit | `initialConfig->setLengthUnit(LengthUnit::METER)` |
| 2 | Organized point cloud | `initialConfig->setOrganized(true)` |
| 3 | Camera-to-camera/housing transform | `setTargetCoordinateSystem(CameraBoardSocket::CAM_A)` |
| 4 | Custom 4×4 transform | `initialConfig->setTransformationMatrix(matrix)` |

### Build & run

```bash
# from the depthai-core root
cmake -S . -B build -DDEPTHAI_BUILD_EXAMPLES=ON
cmake --build build --target pointcloud_showcase -j$(nproc)
./build/examples/cpp/PointCloud/pointcloud_showcase
```

---

## Coordinate-system transforms

The `PointCloud` node can re-express output points in a different coordinate frame
via `setTargetCoordinateSystem()`. Three kinds of target frame are supported:

**A) Camera board socket** — uses extrinsic calibration between cameras.

```cpp
pc->setTargetCoordinateSystem(dai::CameraBoardSocket::CAM_A);
```

Available sockets: `CAM_A` … `CAM_J`.
This preferred overload uses calibrated translations. The legacy
`setTargetCoordinateSystem(socket, useSpecTranslation)` overload is deprecated.

**B) Housing coordinate system** — uses housing calibration stored on the device.

```cpp
pc->setTargetCoordinateSystem(dai::HousingCoordinateSystem::VESA_A);
```

Available targets: `CAM_A`…`CAM_J`, `FRONT_CAM_A`…`FRONT_CAM_J`, `VESA_A`…`VESA_J`, `IMU`.
Housing calibration is resolved using the spec translation path.

**C) Custom 4×4 matrix** — any homogeneous transform via `initialConfig` or the runtime
`inputConfig` queue. When combined with (A)/(B), the custom matrix is applied *after*
the calibration-derived transform.

### Targets on another device

(A) and (B) look the socket or housing up on the device that owns the reference camera of
the depth frame (`Extrinsics::toDeviceId` of the frame extrinsics; with a multi-device
calibration that is the device of the common origin). The overloads with a device ID select
the device explicitly, so the cloud can be expressed in the coordinate system of **any**
camera or housing of **any** device in the pipeline:

```cpp
pc->setTargetCoordinateSystem(deviceB->getDeviceId(), dai::CameraBoardSocket::CAM_C);
pc->setTargetCoordinateSystem(deviceB->getDeviceId(), dai::HousingCoordinateSystem::VESA_A);
```

Resolution chain for a frame whose reference camera lives on another device than the target:
frame → reference camera (frame extrinsics) → local calibration origin of the reference device
→ common origin of the multi-device calibration → local calibration origin of the target
device → target socket / housing (calibration of the target device). The pipeline therefore
needs a multi-device calibration (`Pipeline::setMultiDeviceCalibration`) that connects the two
devices; until it has one, the node logs an error and drops the synced groups instead of
stopping. Every depth stream is resolved on its own, so streams of several devices can be
merged into a cloud expressed in the frame of one of them even before the devices rebase
their frames. The output `Extrinsics` name the target device and socket (housing targets use
`CameraBoardSocket::AUTO`).

Explicit device targets are only supported when the node runs on the host (the default). For
recorded or offline streams the calibration of a device can be supplied with
`setDeviceCalibration(deviceId, calibration)`.

```cpp
std::array<std::array<float, 4>, 4> mat = {{
    {{ 0.f, -1.f, 0.f, 0.f }},
    {{ 1.f,  0.f, 0.f, 0.f }},
    {{ 0.f,  0.f, 1.f, 0.f }},
    {{ 0.f,  0.f, 0.f, 1.f }},
}};
pc->initialConfig->setTransformationMatrix(mat);
```

---

## Merging several depth streams

The node accepts any number of depth streams. The default stream is `inputDepth`; further
streams are created on demand with `getDepthInput(name)` (and `getColorInput(name)` for a
color image aligned to that depth). All streams are synchronized by the internal `sync`
subnode, each is deprojected with its own intrinsics and transformed with its own frame
extrinsics, and the result is sent as **one** `PointCloudData`:

```cpp
auto pc = pipeline.create<dai::node::PointCloud>();
depthA->depth.link(pc->inputDepth);                 // same as pc->getDepthInput("")
depthB->depth.link(pc->getDepthInput("second"));
colorB->link(pc->getColorInput("second"));          // optional, per stream
pc->sync->setSyncThreshold(std::chrono::milliseconds(50));
```

Rules of the merged output:

- Every depth frame has to be expressed relative to the same coordinate system
  (`Extrinsics::toDeviceId` / `Extrinsics::toCameraSocket`). Depth from several devices
  therefore needs a multi-device calibration on the pipeline
  (`Pipeline::setMultiDeviceCalibration`), which makes every device rebase its frame
  extrinsics onto the common origin. Groups whose streams disagree are dropped with a warning
  and merging resumes as soon as they agree again (for example after the calibration is applied
  at runtime).
- The points are ordered by stream (`"depth"` first, then `"depth/<name>"` in alphabetical
  order). Sparse output is a single row with all valid points. Organized output stacks the
  streams row-wise (`width` = stream width, `height` = sum of stream heights) when all streams
  have the same width, otherwise it degrades to a single row.
- The cloud is colorized only when every stream has a usable color frame.
- `setTargetCoordinateSystem()` and custom matrices work as for one stream: the target is
  resolved from the calibration of the device that owns the common reference camera, or of
  the device named in `setTargetCoordinateSystem(deviceId, ...)` (see *Targets on another
  device* above). The groups are merged as soon as every stream resolves to the same output
  coordinate system.
- The output `ImgTransformation` of a merged cloud carries the output size and the coordinate
  system of the points (identity extrinsics to the common origin, or the configured target),
  not the intrinsics of a single source image.
- `passthroughDepth` sends every depth frame of the merged group, in stream order.
- On the host the streams of a group are deprojected concurrently, one thread per stream, each
  into a scratch buffer that is reused from frame to frame; the merged output is then assembled
  with a single allocation. `useCPUMT(n)` still splits every stream over `n` threads on top of
  that (and stays the only parallelism when the node runs on a device). GPU streams are
  processed one after another.
- `useGPU()` computes on the GPU. On an RVC4 device (the node running on the device) this is
  an OpenCL kernel that deprojects, undistorts (through the per-pixel ray table) and transforms
  the points in one pass, with the depth frame imported without a copy where the driver allows
  it. On the host it is the Kompute path when compiled in. The compute method is part of the
  node properties, so `useCPU()` / `useCPUMT(n)` / `useGPU()` apply wherever the node runs; a
  device without a GPU logs a warning and falls back to the CPU.
  Measured on an OAK-4-D (RVC4, board revision P10) for a 1280x800 depth frame: the OpenCL
  kernel itself takes 0.6 ms, but the CPU still has to read the 12 MB of points the GPU wrote
  (to compact and emit them), and on this SoC that read is several times slower than reading
  CPU-written memory. The GPU path therefore ends at about 8.5 ms per cloud against 5.7 ms on
  the CPU, so the CPU stays the default on the device; `useGPU()` there is an option to offload
  the deprojection math, not a speed-up. The node logs the compute time per group at debug
  level every 30 groups, and the GPU backend logs its upload / kernel / map times.
- When the depth streams come from more than one device and the Sync timestamp source is left
  at its default, the Sync subnode is moved to the host at build time.
- Streams only have to be linked, not named in any particular way: a default `inputDepth` that
  stays unlinked next to named streams (and color inputs that were created but never linked)
  are dropped from the Sync subnode when the pipeline starts, so they do not stall the
  synchronization.

A single depth stream behaves exactly as before. See
`examples/python/PointCloud/multi_device_point_cloud.py` for a merged point cloud from several devices.
