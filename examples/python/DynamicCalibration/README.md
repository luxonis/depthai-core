# Dynamic Calibration (Python) — DepthAI

This folder contains minimal, end-to-end examples that use **`dai.node.DynamicCalibration`** to (a) **calibrate** a stereo device in real time, (b) **evaluate** calibration quality on demand, (c) combine both flows, and (d) run a **3-sensor** calibration flow that uses the newer socket-keyed outputs.

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
  - `coveragePerCell: dict[CameraBoardSocket, list[list[float]]]` - per-sensor coverage matrices keyed by socket [0-1].
  - `coveragePerCellA/B: list[list[float]]` - deprecated aliases for the first two connected sensors.

- **DynamicCalibrationResult** (`calibrationOutput`)
  - `calibrationData.newCalibration: dai.CalibrationHandler` — `CalibrationHandler` with updated parameters.
  - `calibrationData.calibrationDifference` — quality deltas between current and new:
    - `pairwiseRotationDifference: dict[tuple[CameraBoardSocket, CameraBoardSocket], list[float]]` — per-sensor-pair rotation deltas keyed by camera socket pairs.
    - `sampsonErrorCurrent: float`, `sampsonErrorNew: float` — reprojection proxy (px).
  - `info` — human-readable status (e.g., "success").

- **CalibrationQuality** (`qualityOutput`, legacy)
  - `qualityData` (optional) — same fields as `calibrationDifference`.
  - `info` — human-readable status.

> `CalibrationQuality` is a legacy command: the node still accepts it for compatibility, but current runtime implementations return an empty result and log a deprecation warning.
> The same metric fields are available on `DynamicCalibrationResult.calibrationData.calibrationDifference`, returned on `calibrationOutput` after `calibrate()` or `startCalibration()`.
> Prefer `pairwiseRotationDifference` in new code. If older code used `rotationChange`, migrate to `pairwiseRotationDifference` by reading the per-sensor-pair deltas directly instead of relying on a single aggregate vector. `rotationChange` is a deprecated compatibility field and should be treated as a candidate for future removal.
> `depthErrorDifference[4]: list[float]` should also be treated as deprecated here and should not be relied on in new code.

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
     dynCalibInputControl.send(dai.DynamicCalibrationControl(dai.DynamicCalibrationControl.Commands.ApplyCalibration(result.calibrationData.newCalibration)))
     ```
   - Print pairwise rotation deltas and Sampson errors.

**Example console output:**
```
2D Spatial Coverage = 0.72 / 100 [%]
Data Acquired = 0.65% / 100 [%]
Dynamic calibration status: success
Successfully calibrated
Pairwise rotation difference (<CAM_B>, <CAM_C>) = [0.12, -0.08, 0.40] deg
Mean Sampson error achievable = 0.21 px
Mean Sampson error current    = 0.38 px
```

> Applying the calibration updates downstream nodes (e.g., `StereoDepth`) immediately for subsequent frames.

---

## 1b) Real-time dynamic calibration with 3 sensors

**Script:** `calibration_dynamic_3_sensors.py`

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
     dynCalibInputControl.send(dai.DynamicCalibrationControl(dai.DynamicCalibrationControl.Commands.CalibrationQuality()))
     dynQualityResult = dynCalibQualityQueue.get()
     ```
3. If `qualityData` is present, print pairwise rotation and Sampson metrics.
4. Optionally reset the internal sample store:
   ```python
   dynCalibInputControl.send(dai.DynamicCalibrationControl(dai.DynamicCalibrationControl.Commands.ResetData()))
   ```

> This flow is kept only as a legacy reference. New code should prefer the calibration output / load-image flow and avoid relying on `CalibrationQuality`.

**Replace this API with:**
- Old: `dai.DynamicCalibrationControl.calibrationQuality()` or `dai.DynamicCalibrationControl.Commands.CalibrationQuality()`
- New for one-shot metrics: `dai.DynamicCalibrationControl.calibrate()`
- New for continuous operation: `dai.DynamicCalibrationControl.startCalibration()`
- Read metrics from: `DynamicCalibrationResult.calibrationData.calibrationDifference` on `calibrationOutput`

Use this only if you need a legacy reference while migrating old code.

---

## 3) Continuous monitoring **+ auto-recalibration** (single script)

**Script:** `calibration_integration.py`

**What it does:**  
Runs one loop that periodically refreshes coverage, executes calibration, and applies a new calibration automatically when the returned metrics indicate drift — while showing `left`, `right`, and colorized `disparity` previews.

**Flow:**
1. Create mono cameras → request **full-res NV12** → link to `DynamicCalibration` and `StereoDepth` for live disparity. Read the device’s current calibration as baseline.
2. On a fixed interval (for example, every ~3 seconds), send:
   - `LoadImage()` to compute coverage on the current frames, and
   - `dai.DynamicCalibrationControl.calibrate(True)` (or equivalent) to compute a new candidate calibration and return metrics on `calibrationOutput`.
3. When a calibration result arrives:
   - If `result.calibrationData` is present, inspect `result.calibrationData.calibrationDifference` for the same rotation and Sampson metrics that older code previously read from `CalibrationQuality.qualityData`.
   - If those metrics indicate drift (e.g., a Sampson error delta above a small threshold like 0.05 px), **apply** the new `CalibrationHandler` to the device.
   - Reset the internal data store before the next monitoring cycle.
4. Treat `result.calibrationData.calibrationDifference` as the replacement source for legacy quality metrics, even if you choose not to apply `result.calibrationData.newCalibration`.
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
                  └───────────▶ StereoDepth.left        calibrationOutput

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

# Live calibration with CAM_A + CAM_B + CAM_C
python calibration_dynamic_3_sensors.py

# Legacy quality evaluation flow
python calibration_quality_dynamic.py

# Integration example (runs calibration, inspects metrics, and applies when needed)
python calibration_integration.py
```

> Tip: Ensure your calibration target is well-lit, sharp, and observed across the **entire** FOV. Aim for high `meanCoverage` and steadily increasing `dataAcquired`.

---

## Commands overview

- `calibrate()`  
  Runs a full calibration immediately and returns metrics on `calibrationOutput`.

- `startCalibration()`  
  Begins dynamic calibration collection/solve.

- `applyCalibration(calibration)`  
  Applies the provided `CalibrationHandler` to the device (affects downstream nodes in this session).

- `loadImage()`  
  Triggers coverage computation for the current frame(s).

- `calibrationQuality()`  
  Legacy command. Produces an empty `CalibrationQuality` result in current runtimes.

If you previously read fields from `CalibrationQuality.qualityData`, read the same fields from `DynamicCalibrationResult.calibrationData.calibrationDifference` after `calibrate()` or `startCalibration()`.

- `resetData()`  
  Clears accumulated samples/coverage state to start fresh.

---

## Metrics glossary

- **Pairwise rotation difference (deg)**  
  Per-sensor-pair rotation deltas keyed by camera socket pairs. This is the preferred current API field for rotation deltas.

- **Sampson error (px)**  
  Proxy for geometric reprojection error. Lower is better.
  - `current` — with the active calibration.
  - `new` — achievable with the proposed calibration.

- **Rotation change (deg, legacy)**  
  Deprecated aggregate rotation delta kept for compatibility. Prefer `pairwiseRotationDifference`.

---

## Troubleshooting

- **No quality data returned**  
  Ensure the pattern is visible, not motion-blurred, and covers diverse regions of the image. Increase lighting, adjust exposure, or hold the rig steady.

- **Disparity looks worse after apply**  
  Re-run to collect more diverse views (tilt/translate the target).

- **Typos in prints**  
  The examples should print “Successfully calibrated” and “Rotation difference” (avoid “Succesfully”/“dofference” if copying code).

---
