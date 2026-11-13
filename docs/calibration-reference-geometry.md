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

## Extended hardware coverage

`camera_sensor_geometry_extended_test` runs in the nightly HIL selection. It
exercises every distinct advertised resolution, sensor type, HDR and HFR variant
using the existing Camera API, including sensor readouts above 16 MP. Large
readouts use smaller output streams to bound host bandwidth. Half-size YUV output
is tested only when its dimensions are even. Each failure is retained while the
extended run continues to assess the remaining modes.

For an explicit device/firmware selection, the same coverage can be run with
`DEPTHAI_GEOMETRY_EXTENDED=1` and the `camera_sensor_geometry_test` executable.
These tests cover geometry at supported selected frame rates, not every FPS,
exposure setting or possible combination of application nodes.
