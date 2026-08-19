# Dynamic Calibration (C++) — DepthAI

This folder contains minimal, end-to-end **C++** examples that use **`dai::node::DynamicCalibration`** to (a) **calibrate** a stereo rig live, (b) **evaluate** calibration quality on demand, (c) combine both flows, and (d) run a **3-sensor** calibration flow that uses the newer socket-keyed outputs.

> Close your preview window or stop the program to quit.

---

## What the node does

`dai::node::DynamicCalibration` consumes **raw, unrectified** left/right mono frames and:
- Streams **coverage feedback**.
- Produces either:
  - a **new calibration** (`CalibrationHandler`) you can apply on the device, or
  - a **quality report** comparing *current* vs *achieved* calibration.

### Outputs & metrics (C++ message types)

- **`CoverageData`** (`coverageOutput`)
  - `meanCoverage` — overall spatial coverage [0–1]
  - `dataAcquired` — amount of calibration-relevant data gathered [0–1]
  - `coveragePerCell` — per-sensor 2D coverage matrices keyed by camera socket
  - `coveragePerCellA/B` — deprecated aliases for the first two connected sensors

- **`DynamicCalibrationResult`** (`calibrationOutput`)
  - `calibrationData->newCalibration` — `CalibrationHandler` with updated parameters
  - `calibrationData->calibrationDifference` — quality deltas:
    - `pairwiseRotationDifference` — per-sensor-pair rotation deltas keyed by camera socket pairs
    - `sampsonErrorCurrent`, `sampsonErrorNew` — reprojection proxy (px)
  - `info` — human-readable status (e.g., "success")

- **`CalibrationQuality`** (`qualityOutput`, legacy)
  - `qualityData` (optional) — same fields as `calibrationDifference`
  - `info` — human-readable status

> `CalibrationQuality` is a legacy command: the node still accepts it for compatibility, but current runtime implementations return an empty result and log a deprecation warning.
> The same metric fields are available on `DynamicCalibrationResult::calibrationData->calibrationDifference`, returned on `calibrationOutput` after `Calibrate` or `StartCalibration`.
> Prefer `pairwiseRotationDifference` in new code. If older code used `rotationChange`, migrate to `pairwiseRotationDifference` by reading the per-sensor-pair deltas directly instead of relying on a single aggregate vector. `rotationChange` is a deprecated compatibility field and should be treated as a candidate for future removal.
> `depthErrorDifference[4]` should also be treated as deprecated here and should not be relied on in new code.

---

## 1) Live dynamic calibration (apply new calibration)

**File idea:** `calibration_dynamic.cpp`

**Flow:**
1. Create mono cameras → request **full-res NV12** (unrectified) → link to:
   - `DynamicCalibration.left/right`
   - `StereoDepth.left/right` (for live depth view)
2. Start the pipeline, give AE a moment to settle.
3. **Start calibration** by sending `DynamicCalibrationControl::Commands::StartCalibration{}`.
4. In the loop:
   - Show `left`, `right`, and `depth`.
   - Poll `coverageOutput` for progress.
   - Poll `calibrationOutput` for a result.
5. When a result arrives:
   - **Apply** it by sending `DynamicCalibrationControl::Commands::ApplyCalibration{result.calibrationData->newCalibration}` on the control queue.
   - Print pairwise rotation deltas and other quality metrics.

---

## 1b) Live dynamic calibration with 3 sensors

**File:** `calibration_dynamic_3_sensors.cpp`

**What it does:**
- Links `CAM_A -> dynCalib.rgb`, `CAM_B -> dynCalib.left`, and `CAM_C -> dynCalib.right`
- Locks the RGB camera to the fixed lens position stored in `CalibrationHandler`
- Starts periodic calibration
- Prints `coveragePerCell` keyed by `CameraBoardSocket`
- Prints `pairwiseRotationDifference` for the calibrated sensor pairs

**Use this when:**
- You want a concrete example of the newer 3-input DynamicCalibration API
- You want to inspect socket-keyed coverage instead of the deprecated `coveragePerCellA/B` aliases

**Important for RGB cameras:**
- Keep the RGB lens position fixed during dynamic calibration.
- Read the RGB lens position from `CalibrationHandler` and apply it with manual focus before starting the pipeline.
- Do not leave RGB autofocus running during calibration, or the RGB camera intrinsics can drift with focus changes.

---

## 2) Calibration **quality check** (legacy)

**File idea:** `calibration_quality_dynamic.cpp`

**Flow:**
1. Same camera / StereoDepth / DynamicCalibration setup as above.
2. In the loop:
   - Send **coverage** request (`DynamicCalibrationControl::Commands::LoadImage{}`) and read `coverageOutput`.
   - Send **quality** request (`DynamicCalibrationControl::Commands::CalibrationQuality{}`) and read `qualityOutput`.
3. If `qualityData` is present, print pairwise rotation and Sampson metrics.
4. Optionally reset internal sample store (`DynamicCalibrationControl::Commands::ResetData{}`).

> This flow is kept only as a legacy reference. New code should prefer the calibration output / load-image flow and avoid relying on `CalibrationQuality`.

**Replace this API with:**
- Old: `DynamicCalibrationControl::Commands::CalibrationQuality{}`
- New for one-shot metrics: `DynamicCalibrationControl::Commands::Calibrate{}`
- New for continuous operation: `DynamicCalibrationControl::Commands::StartCalibration{}`
- Read metrics from: `DynamicCalibrationResult::calibrationData->calibrationDifference` on `calibrationOutput`


---

## 3) Continuous monitoring **+ auto-recalibration** (single binary)

**File:** `calibration_integration.cpp`

**What it does:**  
Runs one loop that periodically refreshes coverage, executes calibration, and applies a new calibration automatically when the returned metrics indicate drift — while showing `left`, `right`, and a colorized `depth` preview.

**Flow:**
1. Create mono cameras → request **full-res NV12** (unrectified) → link to `dai::node::DynamicCalibration` and `dai::node::StereoDepth` for live depth. Read the device’s current calibration as the baseline.
2. On a fixed interval (e.g., ~3 seconds), send on the control queue:
   - `DynamicCalibrationControl::Commands::LoadImage{}` to compute coverage on the current frames, and
   - `DynamicCalibrationControl::Commands::Calibrate{true}` to compute a new candidate calibration and return metrics on `calibrationOutput`.
3. When a **calibration** message arrives on `calibrationOutput`:
   - If `result.calibrationData` is present, inspect `result.calibrationData->calibrationDifference` for the same rotation and Sampson metrics that older code previously read from `CalibrationQuality::qualityData`.
   - If those metrics indicate drift (for example, `abs(sampsonErrorNew - sampsonErrorCurrent) > 0.05f`), **apply** the new `CalibrationHandler` with `DynamicCalibrationControl::Commands::ApplyCalibration{result.calibrationData->newCalibration}`.
   - Reset the internal data store with `DynamicCalibrationControl::Commands::ResetData{}` before the next monitoring cycle.
4. Treat `result.calibrationData->calibrationDifference` as the replacement source for legacy quality metrics, even if you choose not to apply `result.calibrationData->newCalibration`.
5. Exit on `q` keypress or window close.

**Notes & defaults:**
- Depth preview uses the shared colorization helper with a 500–12000 mm range and logarithmic scaling.
- The 0.05 px Sampson threshold is a simple heuristic — tune to your tolerance and noise profile.

**Example console output:**
```
Dynamic calibration status: success
Successfully evaluated metrics from calibration output.
Pairwise rotation difference CAM_B -> CAM_C = [0.12, -0.08, 0.40] deg
Applying new calibration
```
---

## Pipeline diagram

```
Mono CAM_B ──▶ [Camera] ── NV12 (full-res) ──▶ DynamicCalibration.left
                  │                             └─────▶ coverageOutput
                  └───────────▶ StereoDepth.left        calibrationOutput

Mono CAM_C ──▶ [Camera] ── NV12 (full-res) ──▶ DynamicCalibration.right
                  │
                  └───────────▶ StereoDepth.right ──▶ depth
```

---

## Build & run

Requirements:
- C++17
- DepthAI C++ SDK installed (includes headers & libs)
- OpenCV (optional, for visualization)

Example CMakeLists.txt snippet:
```cmake
cmake_minimum_required(VERSION 3.16)
project(dynamic_calib_examples CXX)

find_package(depthai CONFIG REQUIRED)    # provides depthai::core
# find_package(OpenCV REQUIRED)          # if you display frames

add_executable(calibrate_dynamic calibrate_dynamic.cpp)
target_link_libraries(calibrate_dynamic PRIVATE depthai::core)

add_executable(calibration_quality_dynamic calibration_quality_dynamic.cpp)
target_link_libraries(calibration_quality_dynamic PRIVATE depthai::core)

add_executable(calibration_integration calibration_integration.cpp)
target_link_libraries(calibration_integration PRIVATE depthai::core)
```

Build:
```bash
mkdir build && cd build
cmake -DCMAKE_BUILD_TYPE=Release ..
cmake --build . -j
```

Run:
```bash
./calibration_dynamic      # applies new calibration when ready
./calibration_dynamic_3_sensors  # calibrates CAM_A + CAM_B + CAM_C
./calibration_quality_dynamic  # legacy quality-check flow
./calibration_integration   # runs calibration, inspects metrics, and applies when needed
```

---

## Commands overview (C++)

- `DynamicCalibrationControl::Commands::Calibrate{bool force=false, bool keepCameraCenters=true}`  
  Runs a calibration immediately and returns metrics on `calibrationOutput`.

- `DynamicCalibrationControl::Commands::StartCalibration{}`  
  Begins dynamic calibration collection/solve.

- `DynamicCalibrationControl::Commands::ApplyCalibration{CalibrationHandler}`  
  Applies the provided calibration on-device (affects downstream nodes in-session).

- `DynamicCalibrationControl::Commands::LoadImage{}`  
  Triggers coverage computation for the latest frames.

- `DynamicCalibrationControl::Commands::CalibrationQuality{bool force=false}`  
  Legacy command. Produces an empty `CalibrationQuality` result in current runtimes.

If you previously read fields from `CalibrationQuality::qualityData`, read the same fields from `DynamicCalibrationResult::calibrationData->calibrationDifference` after `Calibrate` or `StartCalibration`.

- `DynamicCalibrationControl::Commands::ResetData{}`  
  Clears accumulated samples/coverage state.

---

## Metrics glossary

- **Pairwise rotation difference (deg)** — per-sensor-pair rotation deltas keyed by camera socket pairs. This is the preferred current API field for rotation deltas.  
- **Sampson error (px)** — proxy for geometric reprojection error; lower is better.  
- **Rotation change (deg, legacy)** — deprecated aggregate rotation delta kept for compatibility. Prefer `pairwiseRotationDifference`.

---

## Troubleshooting

- **No quality data returned**  
  Ensure the target is sharp, well-lit, and covers diverse parts of the image. Increase lighting or steady the rig.

- **Depth preview looks worse after apply**
  Collect more diverse samples (tilt/translate the target), or try a performance mode tuned for robustness. Clean lenses; verify focus.

- **Nothing happens after StartCalibration**  
  Make sure you’re feeding **unrectified** mono frames to `DynamicCalibration.left/right`, and give a couple of seconds for AE to settle.
