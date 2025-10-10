# Dynamic Calibration (C++) — DepthAI

This folder contains three minimal, end-to-end **C++** examples that use **`dai::node::DynamicCalibration`** to (a) **calibrate** a stereo rig live and (b) **evaluate** calibration quality on demand, (c) combination of both — while streaming synchronized stereo frames and disparity for visual feedback.

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
  - `coveragePerCellA/B` — 2D coverage matrices (per imager)

- **`DynamicCalibrationResult`** (`calibrationOutput`)
  - `newCalibration` — `CalibrationHandler` with updated parameters
  - `calibrationDifference` — quality deltas:
    - `rotationChange[3]` — extrinsic angle deltas (deg)
    - `sampsonErrorCurrent`, `sampsonErrorNew` — reprojection proxy (px)
    - `depthErrorDifference[4]` — theoretical depth error change at 1/2/5/10 m (%)
  - `info` — human-readable status (e.g., "success")

- **`CalibrationQuality`** (`qualityOutput`)
  - `qualityData` (optional) — same fields as `calibrationDifference`
  - `info` — human-readable status

---

## 1) Live dynamic calibration (apply new calibration)

**File idea:** `calibration_dynamic.cpp`

**Flow:**
1. Create mono cameras → request **full-res NV12** (unrectified) → link to:
   - `DynamicCalibration.left/right`
   - `StereoDepth.left/right` (for live disparity view)
2. Start the pipeline, give AE a moment to settle.
3. **Start calibration** by sending `DynamicCalibrationControl::Command::StartCalibration{}`.
4. In the loop:
   - Show `left`, `right`, and `disparity`.
   - Poll `coverageOutput` for progress.
   - Poll `calibrationOutput` for a result.
5. When a result arrives:
   - **Apply** it by sending `DynamicCalibrationControl::Command::ApplyCalibration{newCalibration}` on the control queue.
   - Print quality deltas.

---

## 2) Calibration **quality check** (no applying)

**File idea:** `calibration_quality_dynamic.cpp`

**Flow:**
1. Same camera / StereoDepth / DynamicCalibration setup as above.
2. In the loop:
   - Send **coverage** request (`DynamicCalibrationControl::Command::LoadImage{}`) and read `coverageOutput`.
   - Send **quality** request (`DynamicCalibrationControl::Command::CalibrationQuality{}`) and read `qualityOutput`.
3. If `qualityData` is present, print rotation/Sampson/depth-error metrics.
4. Optionally reset internal sample store (`DynamicCalibrationControl::Command::ResetData{}`).


---

## 3) Continuous monitoring **+ auto-recalibration** (single binary)

**File:** `calibration_integration.cpp`

**What it does:**  
Runs one loop that periodically checks calibration quality and, if drift is detected, starts calibration and applies the new calibration automatically — while showing `left`, `right`, and a colorized `disparity` preview.

**Flow:**
1. Create mono cameras → request **full-res NV12** (unrectified) → link to `dai::node::DynamicCalibration` and `dai::node::StereoDepth` for live disparity. Read the device’s current calibration as the baseline.
2. On a fixed interval (e.g., ~3 seconds), send on the control queue:
   - `DynamicCalibrationControl::Command::LoadImage{}` to compute coverage on the current frames, and
   - `DynamicCalibrationControl::Command::CalibrationQuality{true}` to request a (forced) quality estimate.
3. When a **quality** message arrives on `qualityOutput`:
   - Log status; if `qualityData` is present, you may **reset** the internal data store with `DynamicCalibrationControl::Command::ResetData{}`.
   - If quality indicates drift (for example, `abs(sampsonErrorNew - sampsonErrorCurrent) > 0.05f`), **start calibration** with `DynamicCalibrationControl::Command::StartCalibration{}`.
4. When a **calibration** message arrives on `calibrationOutput`:
   - **Apply** the new `CalibrationHandler` with `DynamicCalibrationControl::Command::ApplyCalibration{result.newCalibration}`, then **reset** data again (`ResetData{}`) to begin monitoring afresh.
5. Exit on `q` keypress or window close.

**Notes & defaults:**
- Disparity preview can be auto-scaled to the observed maximum; zero disparity can be rendered black for clarity.
- The 0.05 px Sampson threshold is a simple heuristic — tune to your tolerance and noise profile.

**Example console output:**
```
Dynamic calibration status: success
Successfully evaluated Quality
Start recalibration process
Successfully calibrated
```
---

## Pipeline diagram

```
Mono CAM_B ──▶ [Camera] ── NV12 (full-res) ──▶ DynamicCalibration.left
                  │                             └─────▶ coverageOutput
                  └───────────▶ StereoDepth.left        calibrationOutput / qualityOutput

Mono CAM_C ──▶ [Camera] ── NV12 (full-res) ──▶ DynamicCalibration.right
                  │
                  └───────────▶ StereoDepth.right ──▶ disparity
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
./calibration_quality_dynamic  # reports metrics; does not apply
./calibration_integration   # monitors quality, auto-recalibrates, and applies in one loop
```

---

## Commands overview (C++)

- `DynamicCalibrationControl::Command::StartCalibration{}`  
  Begins dynamic calibration collection/solve.

- `DynamicCalibrationControl::Command::ApplyCalibration{CalibrationHandler}`  
  Applies the provided calibration on-device (affects downstream nodes in-session).

- `DynamicCalibrationControl::Command::LoadImage{}`  
  Triggers coverage computation for the latest frames.

- `DynamicCalibrationControl::Command::CalibrationQuality{bool force=false}`  
  Produces a `CalibrationQuality` message with `qualityData` if estimation is possible.

- `DynamicCalibrationControl::Command::ResetData{}`  
  Clears accumulated samples/coverage state.

---

## Metrics glossary

- **Rotation change (deg)** — magnitude of extrinsic delta between current and proposed stereo poses.  
- **Sampson error (px)** — proxy for geometric reprojection error; lower is better.  
- **Depth error difference (%) at 1/2/5/10 m** — theoretical change in relative depth error (negative = improvement).

---

## Troubleshooting

- **No quality data returned**  
  Ensure the target is sharp, well-lit, and covers diverse parts of the image. Increase lighting or steady the rig.

- **Disparity looks worse after apply**  
  Collect more diverse samples (tilt/translate the target), or try a performance mode tuned for robustness. Clean lenses; verify focus.

- **Nothing happens after StartCalibration**  
  Make sure you’re feeding **unrectified** mono frames to `DynamicCalibration.left/right`, and give a couple of seconds for AE to settle.
