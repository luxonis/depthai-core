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

`Depth` supports explicit algorithm selection via `setAlgorithm()` (or `create(device, algorithm)`)
and an `AUTO` mode.

- `AUTO` resolves to:
  - `NEURAL` on RVC4
  - `STEREO` on non-RVC4
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

### Backend accessors

After wiring/build, accessors expose which backend is active:

- `getStereoDepth()`
- `getNeuralDepth()`
- `getNeuralAssistedStereo()`
- `getToF()`
- `getGPUStereo()`

Inactive backends return `nullptr`.

### Configuration helpers

`Depth` provides backend-specific configuration entry points:

- `build(DeviceModelZoo neuralModel)` for NeuralDepth model selection
- `setNeuralAssistedStereoModel(DeviceModelZoo)`
- `setNeuralAssistedStereoRectify(bool)`
- `setTofOptions(CameraBoardSocket, ImageFiltersPresetMode, optional fps)`

## Limitations and constraints

### Device/platform requirements

- `Depth` requires a real device context (`Pipeline(true)` / default device available).
- Stereo-based modes require an available stereo pair on the device.
- `NEURAL_ASSISTED_STEREO` is RVC4-only.
- `GPU_STEREO` is RVC4-only and requires a Kompute-enabled build.
- `TOF` requires a connected ToF camera sensor.

### GPU stereo availability

`GPU_STEREO` additionally depends on product/platform support. On unsupported RVC4 SKUs, validation
fails with an explanatory runtime error.

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

// Optional explicit selection:
// depth->setAlgorithm(dai::node::Depth::Algorithm::NEURAL);

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
