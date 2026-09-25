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
  resolved from the calibration of the device that owns the common reference camera.
- The output `ImgTransformation` of a merged cloud carries the output size and the coordinate
  system of the points (identity extrinsics to the common origin, or the configured target),
  not the intrinsics of a single source image.
- `passthroughDepth` sends every depth frame of the merged group, in stream order.
- When the depth streams come from more than one device and the Sync timestamp source is left
  at its default, the Sync subnode is moved to the host at build time.
- Streams only have to be linked, not named in any particular way: a default `inputDepth` that
  stays unlinked next to named streams (and color inputs that were created but never linked)
  are dropped from the Sync subnode when the pipeline starts, so they do not stall the
  synchronization.

A single depth stream behaves exactly as before. See
`examples/python/MultiDevice/multi_device_point_cloud.py` for a two-device merged point cloud.
