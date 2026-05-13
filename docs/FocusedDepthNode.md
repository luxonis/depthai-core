\page focused_depth_node FocusedDepth node

\tableofcontents

## Purpose

`dai::node::FocusedDepth` (RVC4 only) runs stereo-based depth on a **dynamic region of interest** instead of the full frame. It:

- Synchronizes **left** `ImgFrame`, **right** `ImgFrame`, and a **`FocusedDepthRoi`** message (see `dai::FocusedDepthRoi`).
- Applies a **symmetric horizontal crop** on both eyes: the crop starts at `max(0, roi.x - D)` with width chosen so the union covers the user ROI plus a disparity margin `D` (from `stereoInitialConfig->getMaxDisparity()`), matching standard rectified stereo geometry.
- Runs a **session-stable** backend: **StereoDepth** or **NeuralDepth**, selected at graph build time from `Backend::AUTO` (using `setMaxRoi`, `setTargetFps`) or forced with `setBackend`.

The crop/resize stage uses the same host **ImageManip** path as `dai::node::ImageManip` (`FocusedDepthCrop` subnode, `runOnHost`); intrinsics are updated via `ImgTransformation` on the output frames.

## Inputs

Connect streams to the internal `Sync` inputs (exposed as `left`, `right`, `roi`):

- `left`, `right`: rectified or unrectified mono frames (backend-specific tuning applies as for standalone StereoDepth / NeuralDepth).
- `roi`: `dai::FocusedDepthRoi` carrying `roi` (`dai::Rect`), optional `normalizedCoords`, and `referenceFrame` (v1: `LEFT_RECTIFIED`).

### Synchronization

`Sync` pairs the three streams by timestamp (threshold configurable on `FocusedDepth.sync`). If ROI arrives late (e.g. from an NN), widen the threshold or align pipelines so ROI and stereo frames belong to the same instant.

## Backend selection (AUTO)

Heuristic (conservative, tunable later):

- Large `maxRoiWidth * maxRoiHeight` (above ~576×360) **or** `targetFps` > 35 → **StereoDepth**.
- Otherwise → **NeuralDepth** with a zoo model from `pickNeuralModelForRoi(maxRoiWidth, maxRoiHeight)` (smallest model whose input resolution is ≥ the max ROI in both dimensions), then letterbox resize to that model inside the crop stage.

Override with `setBackend(Backend::STEREO)` / `NEURAL` or `setNeuralModelOverride`.

## Bounding-box sources

Typical producers for `FocusedDepthRoi`:

- Host or **Script**: build `FocusedDepthRoi` from UI or logic.
- **ImgDetections**: map detections to `dai::Rect` (and remap with `ImgDetections::getTransformation().remapRectTo(...)` if needed).
- Other configs (e.g. `SpatialLocationCalculatorConfig` ROIs) can be converted in application code into `FocusedDepthRoi` messages.

## See also

- `include/depthai/pipeline/node/FocusedDepth.hpp`
- `include/depthai/pipeline/datatype/FocusedDepthRoi.hpp`
- `examples/python/RVC4/FocusedDepth/focused_depth_minimal.py`
