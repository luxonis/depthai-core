# RVC4 ToF Examples (C++)

C++ counterparts to the Python examples in [`examples/python/RVC4/ToF/`](../../python/RVC4/ToF/).

| Target | Source | Description |
|--------|--------|-------------|
| `tof_minimal_rvc4` | `tof_minimal.cpp` | Auto-camera + IPP depth preview (OpenCV) |
| `tof_params_minimal_rvc4` | `tof_params_minimal.cpp` | Build, `initialConfig`, runtime `inputConfig` |

Build with the depthai C++ examples (inside dev container):

```bash
cmake --build build --target tof_minimal_rvc4 tof_params_minimal_rvc4
```

Run (device must be RVC4 with ToF):

```bash
./examples/cpp/RVC4/ToF/tof_minimal_rvc4
./examples/cpp/RVC4/ToF/tof_params_minimal_rvc4
```

API notes:
- Use `dai::ToFBuildOptions` with `sensorMode`, `fps`, and optional `preset`
- `getOutputResolution()` — IPP depth/amplitude frame size
- `getRawResolution()` — raw superframe size for manual `Camera.build()`
- `getCamera()` — valid after `pipeline.start()` when auto-camera was used
