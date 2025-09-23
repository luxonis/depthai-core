# Dynamic Calibration (Python) — DepthAI

This folder contains three minimal, end-to-end examples that use **`dai.node.DynamicCalibration`** to (a) **calibrate** a stereo device real-time and (b) **evaluate** calibration quality on demand (c) combination of both — while streaming synchronized stereo frames and disparity for visual feedback.

> Press **`q`** in the preview window to quit either example.

---

## What the node does

`dai.node.DynamicCalibration` consumes **raw, unrectified** left/right mono frames and:
- Streams **coverage feedback**
- Produces either:
  - a **new calibration** (extrinsics) you can apply on the device, or
  - a **quality report** comparing *current* vs *achieved* calibration.

### Outputs & metrics

- **CoverageData** (`coverageOutput`)
  - `meanCoverage: float` — overall spatial coverage [0–1].
  - `dataAcquired: float` — amount of calibration-relevant data gathered [0–1].
  - `coveragePerCellA/B: list[list[float]]` - matrix of spatial coverage of imager A or B [0-1].

- **DynamicCalibrationResult** (`calibrationOutput`)
  - `newCalibration: dai.CalibrationHanlder` — `CalibrationHandler` with updated parameters.
  - `calibrationDifference` — quality deltas between current and new:
    - `rotationChange[3]: list[float]` — extrinsic angle deltas (deg).
    - `sampsonErrorCurrent: float`, `sampsonErrorNew: float` — reprojection proxy (px).
    - `depthErrorDifference[4]: list[float]` — theoretical depth error change at 1/2/5/10 m (%).
  - `info` — human-readable status (e.g., "success").

- **CalibrationQuality** (`qualityOutput`)
  - `qualityData` (optional) — same fields as `calibrationDifference`.
  - `info` — human-readable status.

---

## 1) Real-time dynamic calibration (apply new calibration)

**Script:** `calibration_dynamic.py`

**Flow:**
1. Create mono cameras → request **full-res NV12** (unrectified) → link to:
   - `DynamicCalibration.left/right`
   - `StereoDepth.left/right` (for live disparity view)
2. Start the pipeline, give AE a moment to settle.
3. **Start calibration** with:
   ```python
   dynCalibInputControl.send(
       dai.DynamicCalibrationControl(dai.DynamicCalibrationControl.Commands.StartCalibration())
   )
   ```
4. In the loop:
   - Show `left`, `right`, and `disparity`.
   - Poll `coverageOutput` for progress (`meanCoverage`, `dataAcquired`).
   - Poll `calibrationOutput` for a result.
5. When a result arrives:
   - **Apply** it:
     ```python
     dynCalibInputControl.send(dai.DynamicCalibrationControl(dai.DynamicCalibrationControl.Commands.ApplyCalibration(calibrationData.newCalibration)))
     ```
   - Print quality deltas: rotation magnitude, Sampson errors, depth-error deltas.

**Example console output:**
```
2D Spatial Coverage = 0.72 / 100 [%]
Data Acquired = 0.65% / 100 [%]
Dynamic calibration status: success
Successfully calibrated
Rotation difference: || r_current - r_new || = 0.43 deg
Mean Sampson error achievable = 0.21 px
Mean Sampson error current    = 0.38 px
Theoretical Depth Error Difference @1m:-4.20%, 2m:-3.10%, 5m:-1.60%, 10m:-0.90%
```

> Applying the calibration updates downstream nodes (e.g., `StereoDepth`) immediately for subsequent frames.

---

## 2) Calibration **quality check**

**Script:** `calibration_quality_dynamic.py`

**Flow:**
1. Same camera / StereoDepth / DynamicCalibration setup as above.
2. In the loop:
   - Show `left`, `right`, and `disparity`.
   - Ask for **coverage** on demand:
     ```python
     dynCalibInputControl.send(dai.DynamicCalibrationControl(dai.DynamicCalibrationControl.Commands.LoadImage()))
     coverage = dynCalibCoverageQueue.get()
     ```
   - Ask for **quality** on demand:
     ```python
     dynCalibInputControl.send(dai.DynamicCalibrationControl(dai.DynamicCalibrationControl.Commands.dai.CalibrationQuality()))
     dynQualityResult = dynCalibQualityQueue.get()
     ```
3. If `qualityData` is present, print rotation/Sampson/depth-error metrics.
4. Optionally reset the internal sample store:
   ```python
   dynCalibInputControl.send(dai.DynamicCalibrationControl(dai.DynamicCalibrationControl.Commands.ResetData()))
   ```

**Use this when:**
- You want to **assess** a potential calibration before committing it.
- You’re tuning capture/coverage practices and need fast feedback.

---

## 3) Continuous monitoring **+ auto-recalibration** (single script)

**Script:** `calibration_integration.py`

**What it does:**  
Runs one loop that periodically checks calibration quality and, if drift is detected, starts calibration and applies the new calibration automatically — while showing `left`, `right`, and colorized `disparity` previews.

**Flow:**
1. Create mono cameras → request **full-res NV12** → link to `DynamicCalibration` and `StereoDepth` for live disparity. Read the device’s current calibration as baseline.
2. On a fixed interval (for example, every ~3 seconds), send:
   - `LoadImage()` to compute coverage on the current frames, and
   - `CalibrationQuality(True)` (or equivalent) to request a quality estimate.
3. When a quality result arrives:
   - Log status; if quality data is present, **reset** the internal data store.
   - If the quality indicates drift (e.g., a Sampson error delta above a small threshold like 0.05 px), **start calibration** (`StartCalibration()`).
4. When a calibration result arrives:
   - **Apply** the new `CalibrationHandler` to the device and **reset** data again.
5. Press **`q`** to exit.

**Notes & defaults:**
- Disparity preview is auto-scaled to the observed maximum; **zero disparity appears black** for clarity.
- The 0.05 px Sampson threshold is a simple heuristic — adjust per your tolerance.

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

## Requirements

- Python 3.8+
- `depthai`, `opencv-python`, `numpy`

Install:
```bash
pip install depthai opencv-python numpy
```

---

## Run

```bash
# Live calibration (applies new calibration when ready)
python calibration_dynamic.py

# Quality evaluation (reports metrics of when recalibration is required)
python calibration_quality_dynamic.py

# Integration example (starts new calibration when the quality reports major changes)
python calibration_integration.py
```

> Tip: Ensure your calibration target is well-lit, sharp, and observed across the **entire** FOV. Aim for high `meanCoverage` and steadily increasing `dataAcquired`.

---

## Commands overview

- `StartCalibration()`  
  Begins dynamic calibration collection/solve.

- `ApplyCalibration(calibration)`  
  Applies the provided `CalibrationHandler` to the device (affects downstream nodes in this session).

- `LoadImage()`  
  Triggers coverage computation for the current frame(s).

- `CalibrationQuality()`  
  Produces a `CalibrationQuality` message with `qualityData` if estimation is possible.

- `ResetData()`  
  Clears accumulated samples/coverage state to start fresh.

---

## Metrics glossary

- **Rotation change (deg)**  
  Magnitude of the delta between current and new stereo extrinsics (roll/pitch/yaw).

- **Sampson error (px)**  
  Proxy for geometric reprojection error. Lower is better.
  - `current` — with the active calibration.
  - `new` — achievable with the proposed calibration.

- **Depth error difference (%) at 1/2/5/10 m**  
  Theoretical change in relative depth error. **Negative values** indicate improvement.

---

## Troubleshooting

- **No quality data returned**  
  Ensure the pattern is visible, not motion-blurred, and covers diverse regions of the image. Increase lighting, adjust exposure, or hold the rig steady.

- **Disparity looks worse after apply**  
  Re-run to collect more diverse views (tilt/translate the target).

- **Typos in prints**  
  The examples should print “Successfully calibrated” and “Rotation difference” (avoid “Succesfully”/“dofference” if copying code).

---
