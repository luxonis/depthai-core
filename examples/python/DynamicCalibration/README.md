# Dynamic Calibration (Python) — DepthAI

This folder contains two minimal, end-to-end examples that use **`dai.node.DynamicCalibration`** to (a) **calibrate** a stereo rig live and (b) **evaluate** calibration quality on demand — both while streaming synchronized stereo frames and disparity for visual feedback.

> Press **`q`** in the preview window to quit either example.

---

## What the node does

`dai.node.DynamicCalibration` consumes **raw, unrectified** left/right mono frames and:
- Streams **coverage feedback** (how well the calibration pattern is seen across the image).
- Produces either:
  - a **new calibration** (intrinsics/extrinsics) you can apply on the device, or
  - a **quality report** comparing *current* vs *achievable* calibration.

### Outputs & metrics

- **CoverageData** (`coverageOutput`)
  - `meanCoverage` — overall spatial coverage [0–1].
  - `dataAcquired` — amount of calibration-relevant data gathered [0–1].
  - `coveragePerCellA/B` - matrix of spatial coverage of imager A or B

- **DynamicCalibrationResult** (`calibrationOutput`)
  - `newCalibration` — `CalibrationHandler` with updated parameters.
  - `calibrationDifference` — quality deltas between current and new:
    - `rotationChange[3]` — extrinsic angle deltas (deg).
    - `sampsonErrorCurrent`, `sampsonErrorNew` — reprojection proxy (px).
    - `depthErrorDifference[4]` — theoretical depth error change at 1/2/5/10 m (%).
  - `info` — human-readable status (e.g., "success").

- **CalibrationQuality** (`qualityOutput`)
  - `qualityData` (optional) — same fields as `calibrationDifference`.
  - `info` — human-readable status.

---

## 1) Live dynamic calibration (apply new calibration)

**Script idea:** `calibrate_dynamic.py` (based on your first snippet)

**Flow:**
1. Create mono cameras → request **full-res NV12** (unrectified) → link to:
   - `DynamicCalibration.left/right`
   - `StereoDepth.left/right` (for live disparity view)
2. Start the pipeline, give AE a moment to settle.
3. **Start calibration** with:
   ```python
   dynCalibInputControl.send(
       dai.StartCalibrationCommand()
   )
   ```
4. In the loop:
   - Show `left`, `right`, and `disparity`.
   - Poll `coverageOutput` for progress (`meanCoverage`, `dataAcquired`).
   - Poll `calibrationOutput` for a result.
5. When a result arrives:
   - **Apply** it:
     ```python
     dynCalibInputControl.send(dai.ApplyCalibrationCommand(calibrationData.newCalibration))
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

## 2) Calibration **quality check** (no applying)

**Script idea:** `quality_dynamic.py` (based on your second snippet)

**Flow:**
1. Same camera / StereoDepth / DynamicCalibration setup as above.
2. In the loop:
   - Show `left`, `right`, and `disparity`.
   - Ask for **coverage** on demand:
     ```python
     dynCalibInputControl.send(dai.LoadImageCommand())
     coverage = dynCalibCoverageQueue.get()
     ```
   - Ask for **quality** on demand:
     ```python
     dynCalibInputControl.send(dai.CalibrationQualityCommand())
     dynQualityResult = dynCalibQualityQueue.get()
     ```
3. If `qualityData` is present, print rotation/Sampson/depth-error metrics.
4. Optionally reset the internal sample store:
   ```python
   dynCalibInputControl.send(dai.ResetDataCommand())
   ```

**Use this when:**
- You want to **assess** a potential calibration before committing it.
- You’re tuning capture/coverage practices and need fast feedback.

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
python calibrate_dynamic.py

# Quality evaluation (reports metrics; does not apply)
python quality_dynamic.py
```

> Tip: Ensure your calibration target is well-lit, sharp, and observed across the **entire** FOV. Aim for high `meanCoverage` and steadily increasing `dataAcquired`.

---

## Commands overview

- `StartCalibrationCommand()`  
  Begins dynamic calibration collection/solve. Use `OPTIMIZE_SPEED` for speed or `OPTIMIZE_PERFORMANCE` for more robust estimation.

- `ApplyCalibrationCommand(calibration)`  
  Applies the provided `CalibrationHandler` to the device (affects downstream nodes in this session).

- `LoadImageCommand()`  
  Triggers coverage computation for the current frame(s).

- `CalibrationQualityCommand()`  
  Produces a `CalibrationQuality` message with `qualityData` if estimation is possible.

- `ResetDataCommand()`  
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
  Re-run to collect more diverse views (tilt/translate the target), or try `OPTIMIZE_PERFORMANCE`. Check for lens smudges and ensure focus is appropriate.

- **Typos in prints**  
  The examples should print “Successfully calibrated” and “Rotation difference” (avoid “Succesfully”/“dofference” if copying code).

---