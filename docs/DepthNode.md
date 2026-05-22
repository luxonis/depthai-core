# Depth composite node

The `dai::node::Depth` node provides a single API over multiple depth backends (StereoDepth, NeuralDepth, NeuralAssistedStereo, ToF, GPUStereo).

## RVC4 algorithm selection

`Algorithm::AUTO` runs one pass in `Depth::selectBackend(resolution, targetFps, …)`:

**Inputs**

- **Target FPS** (always applied): `build(fps)` → upstream stereo cameras' max requested FPS → **30**.
- **Resolution** (optional gate): `build(..., stereoSize)` → upstream stereo cameras' configured size → unset (no resolution check).

**Rules** (hard-coded, single pass)

1. **NeuralDepth** — walk models largest-first. Pick the first model whose `maxFps` covers `targetFps × 0.85`. When resolution is set, the model must also cover that frame size (per-axis ≈ tensor ± √2).
2. **NeuralAssistedStereo** — if step 1 found nothing, require the same FPS budget and resolution ≤ 1280×1280 (no selectable profile; always uses `NEURAL_DEPTH_NANO` internally).
3. **StereoDepth** — quality-ordered presets (`ACCURACY` … `FAST_ACCURACY`) with the same FPS and resolution rules.
4. **GPUStereo** — if step 3 found nothing, require FPS only (any resolution).

Each result is an `Algorithm` plus a `Config` variant: `DeviceModelZoo` for `NEURAL`, `StereoDepth::PresetMode` for `STEREO`, `std::monostate` for NAS / GPUStereo / ToF.

Non-RVC4 `AUTO`: ToF when a ToF sensor is connected on RVC2, otherwise `StereoDepth`.

## API

```cpp
auto depth = pipeline.create<dai::node::Depth>();
depth->build(dai::node::Depth::Algorithm::AUTO, 30.f, std::pair<uint32_t, uint32_t>{640, 400});

(void)depth->depth();  // triggers wiring

depth->getResolvedAlgorithm();
depth->getResolvedConfig();           // variant
depth->getResolvedNeuralModel();      // NEURAL / NAS convenience
depth->getResolvedStereoPreset();     // STEREO convenience
```

Static helpers:

- `Depth::selectBackend(resolution, targetFps, supportedAlgorithms, supportedModels = {})`
- `Depth::getSupportedAlgorithms(device)` — single source of truth for which backends are available
- `Depth::exceedsStereoDepthMaxResolution(width, height)`
