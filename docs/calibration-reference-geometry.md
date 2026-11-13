# Calibration image reference geometry

Calibration intrinsics describe the pixels of the image used for calibration.
Its width and height alone do not identify a sensor readout or physical field of view:
a native crop and a resized full-sensor image can have the same dimensions.

Existing `setCameraIntrinsics` and `getCameraIntrinsics` retain their behavior.
The latter is a generic resize/crop operation; it does not select a physical sensor mode.
Camera firmware maps the calibration reference to the selected sensor readout.
Unlabelled records keep the platform's historical interpretation. In particular,
RVC4 assumes reduced-resolution calibration represents the full sensor FOV under
the existing aspect-ratio resize/crop rules. RVC2 retains its existing normalization
for recognized native sensor calibration modes.

When the calibration images' physical crop and scale are known, use
`setCameraIntrinsicsFromSensorCrop` to store K in full physical sensor coordinates.
This is explicit and does not change other calibration records or write EEPROM.
Use the actual sensor's advertised geometry; the numbers below illustrate an
unbinned 3840×2160 crop at (184,480) of a 4208×3120 sensor.

```python
calibration = device.readCalibration()
calibration.setCameraIntrinsicsFromSensorCrop(
    dai.CameraBoardSocket.CAM_A,
    measured_K, 3840, 2160, 4208, 3120,
    dai.Rect(184, 480, 3840, 2160),
)
pipeline.setCalibrationData(calibration)
```

Equivalent C++:

```cpp
auto calibration = device->readCalibration();
calibration.setCameraIntrinsicsFromSensorCrop(
    dai::CameraBoardSocket::CAM_A, measuredK,
    3840, 2160, 4208, 3120, dai::Rect(184, 480, 3840, 2160));
pipeline.setCalibrationData(calibration);
```

For each axis, the helper undoes the calibration-image scale and adds the ROI
origin. For example, `fx_sensor = fx_image * crop.width / imageWidth` and
`cx_sensor = cx_image * crop.width / imageWidth + crop.x`. It scales whole K rows,
including skew. It stores the result with the full sensor width and height.
Distortion coefficients, model and camera extrinsics remain unchanged.

The crop covers the entire calibration image and uses absolute physical pixels.
Include both sensor readout and later crop/resize when calculating it. Binning is
represented by a physical crop larger than the calibration image. The helper does
not support padding, rotation, undistortion or rectification. Do not recover capture
geometry from frame metadata known to be affected by a geometry bug.

For processed output, obtain intrinsics from that frame:

```python
K = frame.getTransformation().getIntrinsicMatrix()
```

```cpp
auto K = frame->getTransformation().getIntrinsicMatrix();
```

Only normalize when the capture geometry is known. Do not infer native capture
from matching image dimensions, and do not normalize an already-normalized record
again. A known-native old JSON can be converted in memory and used for the session;
existing EEPROM contents do not need to be rewritten.
