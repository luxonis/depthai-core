# RVC2 ToF Examples (Myriad)

Python examples for `dai.node.ToF` on **RVC2** (OAK-D ToF and similar).

On RVC2:
- **Host ImageFilters** — `tof.depth` is filtered; `tof.rawDepth` is unfiltered ToFBase depth
- **Build** — `tof.build(socket, presetMode=ImageFiltersPresetMode, fps=...)`
- **`tof.confidence`** — aliases `tof.amplitude` (per-pixel quality map)

## Examples

| Script | Purpose |
|--------|---------|
| [`tof_minimal.py`](tof_minimal.py) | Raw vs filtered depth preview |
| [`tof_confidence.py`](tof_confidence.py) | `confidence` = `amplitude` on RVC2 |
| [`tof_align.py`](tof_align.py) | RGB + aligned ToF depth with sync |

## C++ counterparts

See [`examples/cpp/RVC2/ToF/`](../../cpp/RVC2/ToF/).

```bash
cd external/depthai-core
python examples/python/RVC2/ToF/tof_minimal.py
```
