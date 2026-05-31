# Depth composite node

The `dai::node::Depth` node provides a single API over multiple depth backends (StereoDepth, NeuralDepth, NeuralAssistedStereo, ToF, GPUStereo).

## RVC4 algorithm selection

`Algorithm::AUTO` runs one pass in `Depth::selectBackend(resolution, targetFps, …)`:

**Inputs**

- **Target FPS** (always applied): `build(fps)` → upstream stereo cameras' max requested FPS → **30**.
- **Resolution** (always derived): `build(..., stereoSize)` → upstream stereo cameras' configured size → device's `getConnectedCameraFeatures()` for the stereo pair sockets (sensor native). Set to unknown only when none of those sources advertise a size.

**Rules** (single scan over one unified `BackendProfile` catalog)

Every backend has one (or several) row(s) in `kBackendProfiles` in `src/pipeline/node/Depth.cpp`. Each row is `{algorithm, config, maxWidth, maxHeight, maxFps}`. Selection walks rows in this priority order and returns the first row whose algorithm is in `supportedAlgorithms`, whose `maxFps ≥ targetFps × 0.85`, and (when a resolution is provided) whose resolution rule passes:

1. **NeuralDepth** — 10 rows, largest tensor first. Resolution rule is a tensor-cover band (≈ tensor ± √2 per axis). Model must also appear in `supportedModels` when that filter is non-empty.
2. **NeuralAssistedStereo** — single row: **1280×800 @ 55 FPS** (no selectable profile; always uses `NEURAL_DEPTH_NANO` internally).
3. **StereoDepth** — 8 quality-ordered presets (`ACCURACY` / `HIGH_DETAIL` @ 15 → `DENSITY` / `DEFAULT` / `FACE` / `ROBOTICS` @ 30 → `FAST_DENSITY` / `FAST_ACCURACY` @ 60), all capped at 1280×1280.
4. **GPUStereo** — 3 rows: **2592×1944 @ 5.5 FPS**, **1280×800 @ 34 FPS**, **640×400 @ 60 FPS**.

If no row qualifies the priority scan, a second pass picks the catalog row that **covers the requested resolution with the highest available `maxFps`** (resolution-fit fallback). This keeps high-resolution sensors (e.g. 2592×1944) on `GPUStereo` rather than dropping back to a `StereoDepth` preset that cannot serve the resolution. If no row covers the resolution at all, the result is the fastest `StereoDepth` preset (or `{STEREO, FAST_ACCURACY}` as a last resort).

Each result is an `Algorithm` plus a `Config` variant: `DeviceModelZoo` for `NEURAL`, `StereoDepth::PresetMode` for `STEREO`, `std::monostate` for NAS / GPUStereo / ToF. (For GPUStereo with multiple resolution rows, `Config` does not currently expose which row matched.)

Non-RVC4 `AUTO`: ToF when a ToF sensor is connected on RVC2, otherwise `StereoDepth`.

## API

```cpp
auto depth = pipeline.create<dai::node::Depth>();
depth->build(dai::node::Depth::Algorithm::AUTO, 30.f, std::pair<uint32_t, uint32_t>{640, 400});

(void)depth->depth();  // triggers wiring

depth->getResolvedAlgorithm();
depth->getResolvedPreset();           // variant: model, preset, or std::monostate
```

Python: `pipeline.create(dai.node.Depth, ...)` accepts the same selection hints as keyword arguments and forwards them to `Depth::build(...)` before lazy wiring runs (so AUTO sees them on the first scan):

- `fps=` / `FPS=` — target stereo FPS (same as `build(fps)`)
- `resolution=` / `stereo_size=` — `(width, height)` tuple used by AUTO selection when no upstream `Camera` is wired
- `algorithm=` — pin a specific `dai.node.Depth.Algorithm` (skip AUTO)

```python
depth = pipeline.create(dai.node.Depth, FPS=10.0, resolution=(640, 400))  # AUTO → NeuralDepth on RVC4
```

Static helpers:

- `Depth::selectBackend(resolution, targetFps, supportedAlgorithms, supportedModels = {})`
- `Depth::getSupportedAlgorithms(device)` — single source of truth for which backends are available
- `Depth::exceedsStereoDepthMaxResolution(width, height)`
