# Correctness and nodes

Use when the graph runs but outputs are incorrect. Capture payload and metadata
at the first boundary where expected and actual results diverge.

## Controlled input

Use recorded/synthetic input with a known result. Save its hash, node/model
configuration, sequence, timestamps, dimensions, format, stride, transformations
and calibration. Compare input, passthrough and output; normal FPS can accompany
empty or invalid data.

Start from matching examples in `examples/{cpp,python}` and tests in
`tests/src/{onhost_tests,ondevice_tests}`. Preserve configuration while removing
unrelated nodes. Compare C++/Python when conversion/binding behavior is plausible.
See [performance](performance-and-resources.md) for replay limits.

| Area | Checks and discriminating evidence |
| --- | --- |
| Camera | Sensor/socket identity; requested/effective mode, crop/resize, FPS/exposure, frame type and sequence. Compare one output then simultaneous outputs. On RVC4 trace correct SDK requests into camera topology. |
| Stereo/depth | Input timing, calibration, rectification, left/right assignment, confidence, LR/extended/subpixel settings, disparity range and units. Save matching left/right, disparity, depth and invalid-pixel masks. Use ground truth for accuracy. |
| ImageManip/ImgFrame | RGB/BGR, planar/interleaved, strides/padding, type, dimensions, ROI, rotation, resize and transformation metadata. Inspect a small known pattern before a full model pipeline. |
| NeuralNetwork | Model/NNArchive hash, platform/runtime, tensor shape/layout/type, normalization, quantization, names and raw outputs. Separate inference from preprocessing and parsing. |
| Detection/tracking/spatial | Parser settings, passthrough/sequence correspondence, ROI mapping, tracker inputs, alignment, coordinates and units. Compare raw detections first. |
| IMU/sync/time | Batching, units, timestamp origin, rate/drops, extrinsics and FSYNC/PTP configuration. Correlate clocks before comparing latency. |
| Encoder/streaming | Codec/profile, resolution/FPS, bitrate/keyframes, packet integrity, decoder and sink speed. Save a stream to separate encoding from display/network delivery. |
| Script/host node | Exceptions, blocking get/send, config availability, thread lifetime, GIL and ownership. Inspect all relevant threads for hangs. |
| Calibration/point cloud | Runtime versus EEPROM calibration, intrinsics/extrinsics, transform direction, units and registration. Check a known point/transform first. |
| Models/integrations | Cache/requested model version, optional libraries/GPU, download errors and visualizer overhead. Establish bytes and compute output independently of GUI. |

## Timestamp discipline

Preserve device timestamps, host-synchronized timestamps and arrival time as
different measurements. Use monotonic durations. Do not subtract timestamps from
unrelated boots or clock domains. Track sequence numbers too: drops and reused
frames otherwise resemble drift. Treat uptime/sequence resets after reconnect
explicitly.

## Shared types and configuration

Compare shared definitions and serialization on both sides, including the SDK
dependency inside firmware. Inspect enum mappings, field defaults, protobufs
and running firmware. A successful API call does not establish agreement on
representation. Any shared-type fix needs the repository's compatibility rules,
paired firmware artifacts and behavioral/roundtrip tests.

## Calibration and quality

Read/export calibration first. Distinguish runtime overrides from persistent
flashing. `DEPTHAI_AUTOCALIBRATION` can lead to EEPROM writes at this revision;
do not enable it just to inspect calibration.

Use bit-exact comparisons when output should stay unchanged. For intended quality
changes, use reference calculations or ground truth, report invalid/abstained
results, and specify tolerances. Smoothness or agreement with a differently
configured algorithm is not an accuracy test.

Sources: [datatypes](../../../../include/depthai/pipeline/datatype),
[serialization](../../../../src/utility/Serialization.cpp),
[calibration](../../../../include/depthai/device/CalibrationHandler.hpp),
[examples](../../../../examples), [consistency checks](../../../../ci).
