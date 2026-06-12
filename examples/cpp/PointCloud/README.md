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
// useSpecTranslation=true  →  use nominal spec translation instead of per-unit calibration
```

Available sockets: `CAM_A` … `CAM_J`. `useSpecTranslation` defaults to `false`.

**B) Housing coordinate system** — uses housing calibration stored on the device.

```cpp
pc->setTargetCoordinateSystem(dai::HousingCoordinateSystem::VESA_A);
// useSpecTranslation=false  →  use per-unit calibrated values instead of spec
```

Available targets: `CAM_A`…`CAM_J`, `FRONT_CAM_A`…`FRONT_CAM_J`, `VESA_A`…`VESA_J`, `IMU`.
`useSpecTranslation` defaults to `true`.

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
