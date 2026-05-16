\page depth_node Depth Node

\tableofcontents

## Purpose

`dai::node::Depth` is a composite node that provides a single "depth API surface" and internally
selects and wires one of several depth backends:

- `StereoDepth`
- `NeuralDepth`
- `NeuralAssistedStereo`
- `ToF`
- `GPUStereo`

The main goal is to let applications request `depth()` / `confidence()` from one node while keeping
algorithm selection and camera wiring logic inside DepthAI.

## Functionality

### Backend selection

`Depth` supports explicit algorithm selection via `Pipeline::create<Depth>(algorithm)` / `Depth::create(algorithm)` / `explicit Depth(Algorithm)`,
default `AUTO` via `Pipeline::create<Depth>()` / `Depth()` / `Depth::create()`, and an `AUTO` mode that resolves when the graph is wired.
Before first wiring, the algorithm can also be changed through `build(algorithm, ...)`.

- `AUTO` resolves to:
  - `NEURAL` on RVC4
  - `TOF` on RVC2 when a connected camera reports `CameraSensorType::TOF` in `getConnectedCameraFeatures()`
  - `STEREO` on other platforms (and on RVC2 when no ToF sensor is reported)
- Explicit values are validated against device/platform constraints before backend wiring.

### Lazy wiring

Backend graph construction is lazy:

- The internal graph is built on first `depth()` / `confidence()` access.
- `pipeline.build()` then validates/finalizes the full graph as usual.

### Camera reuse

For stereo-based algorithms (`STEREO`, `NEURAL`, `GPU_STEREO`, `NEURAL_ASSISTED_STEREO`):

- `Depth` tries to find already-created `Camera` nodes for the first stereo pair.
- If matching cameras are found, they are reused.
- If missing, `Depth` creates required stereo cameras automatically.

This allows both common wiring styles:

- user creates left/right cameras before creating `Depth`
- user creates `Depth` first and cameras later

## Features

### Unified outputs

- `depth()` always exposes the active backend depth output.
- `confidence()` maps to backend-specific confidence-like output:
  - `StereoDepth`: `confidenceMap`
  - `NeuralDepth`: `confidence`
  - `NeuralAssistedStereo`: stereo confidence map
  - `ToF`: `amplitude`
  - `GPUStereo`: `disparity`

Algorithm implementation nodes (`StereoDepth`, `NeuralDepth`, and so on) are not part of the public `Depth` API; they are created when the graph is first wired with fixed defaults (neural zoo model, ToF socket/preset, NAS rectify flag, and so on). The only user-controlled knob is `Algorithm`, set either at construction or through `build(algorithm, ...)` before first wiring.

### Algorithm configuration

- Use `Pipeline::create<Depth>(algorithm)`, `Depth::create(…)`, or `explicit Depth(Algorithm)` to pick the backend in C++, or `depthNode.build(dai.node.Depth.Algorithm.…)` before first wiring in Python. The C++ factory `dai::node::Depth::create` accepts a null device pointer: the pipeline's default device is applied when the node is added. `Pipeline::create<Depth>()` passes the pipeline default device. For automatic algorithm selection, use `Depth::create()`, `Pipeline::create<Depth>()`, or a default-constructed `Depth()` (all use `AUTO`).
- For per-backend tuning (NeuralDepth model, ToF preset, StereoDepth presets, and so on), use the dedicated node types directly instead of `Depth`.

## Limitations and constraints

### Device/platform requirements

- `Depth` requires a real device context (`Pipeline(true)` / default device available).
- Stereo-based algorithms (`STEREO`, `NEURAL`, `GPU_STEREO`, `NEURAL_ASSISTED_STEREO`) require an available stereo pair on the device. `TOF` and `AUTO` when it resolves to `TOF` do not.
- `NEURAL_ASSISTED_STEREO` is RVC4-only.
- `GPU_STEREO` is RVC4-only.
- `TOF` requires a connected ToF camera sensor.

### Backend-specific semantics

Even with unified `depth()` / `confidence()` accessors, behavior depends on the selected backend:

- confidence meaning, scale, and image semantics are backend-specific
- available tuning knobs differ between algorithms
- output size/FPS depend on backend and camera request path

Applications that need stable cross-backend behavior should validate output expectations explicitly.

## Typical usage

### C++

```cpp
dai::Pipeline pipeline;
auto depth = pipeline.create<dai::node::Depth>();
// Or explicit backend at creation:
// auto depth = pipeline.create<dai::node::Depth>(dai::node::Depth::Algorithm::NEURAL);

auto depthQ = depth->depth().createOutputQueue();
auto confQ = depth->confidence().createOutputQueue();

pipeline.build();
```

### Python

```python
pipeline = dai.Pipeline()
depth_node = pipeline.create(dai.node.Depth)

depth_q = depth_node.depth.createOutputQueue()
conf_q = depth_node.confidence.createOutputQueue()

pipeline.build()
```

## See also

- `include/depthai/pipeline/node/Depth.hpp`
- `src/pipeline/node/Depth.cpp`
- `tests/src/ondevice_tests/pipeline/node/depth_node_test.cpp`
