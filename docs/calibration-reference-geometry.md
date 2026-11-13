# Camera calibration compatibility

This fix requires no new public API or changes to existing calibration files.
Existing `setCameraIntrinsics` and `getCameraIntrinsics` retain their behavior.
The latter remains a generic image resize/crop helper and does not select a
physical sensor readout.

RVC2 retains its existing normalization of calibration records. For example,
both a full 4208×3120 IMX214 calibration and an existing native 3840×2160
calibration are mapped through the historical normalization into physical sensor
coordinates. The corrected output mapping then applies the selected readout's
crop and bin/scale once. For that native unbinned crop, focal lengths remain
unchanged and the principal point is shifted by its physical crop origin.
No recalibration, new call or EEPROM rewrite is needed for these cases.

RVC4 retains its historical interpretation of reduced-resolution calibration
using the existing full-sensor resize/crop rules. The selected sensor crop is
applied consistently at startup and on runtime calibration updates. Calibration
image dimensions are not reinterpreted as proof of a native capture mode.

A calibration's width and height alone cannot distinguish a native sensor crop
from a processed image of the same size. The fix preserves existing interpretation;
it does not automatically repair arbitrary files with unknown capture provenance.

Camera frame metadata follows the actual output crop and scale. Code consuming
processed frames can use the existing per-frame transformation API:

```python
K = frame.getTransformation().getIntrinsicMatrix()
```

```cpp
auto K = frame->getTransformation().getIntrinsicMatrix();
```

The firmware change corrects metadata for affected readouts without changing
image pixels or the generic CalibrationHandler API contract.
