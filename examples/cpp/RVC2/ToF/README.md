# RVC2 ToF Examples (C++)

C++ ToF examples for **RVC2** devices (Myriad + host ImageFilters).

| Target | Source | Description |
|--------|--------|-------------|
| `tof_minimal` | `tof_minimal.cpp` | `rawDepth` vs filtered `depth` preview |
| `tof_align` | `tof_align.cpp` | RGB + ImageAlign + optional confidence filter |

Build:

```bash
cmake --build build --target tof_minimal tof_align
```

On RVC2, `tof->confidence` references the same output as `tof->amplitude`.

Python versions: [`examples/python/RVC2/ToF/`](../../python/RVC2/ToF/).
