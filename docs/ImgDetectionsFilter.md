# ImgDetectionsFilter

`dai::node::ImgDetectionsFilter` / `dai.node.ImgDetectionsFilter` filters one camera's detections or combines detections from multiple cameras in a common image. It replaces the beta node and config, which have been removed.

For one camera, link `DetectionNetwork.out` (or `DetectionParser.out`) to `filter.inputs["cam"]`. Without a reference, surviving detections keep their original coordinates and metadata. The default config preserves every detection. See [Python](../examples/python/NeuralNetwork/img_detections_filter.py) and [C++](../examples/cpp/NeuralNetwork/img_detections_filter.cpp).

For multiple cameras, synchronize detections with a host `Sync`, then link `MessageDemux.outputs[key]` to each `filter.inputs[key]`. Link the stitched image to `inputReference`, or assign an `ImgTransformation` to `initialConfig.reference`. At least one data input must be linked. Multiple inputs require a reference source. Unlinked keys have no effect.

The filter consumes one message from every linked key for each round. It assumes upstream synchronization and does not compare timestamps. If one input loses a message, subsequent rounds can refer to different moments. Keep the data inputs blocking and synchronize upstream. The filter waits for a complete round, including messages containing no detections.

`inputReference` is nonblocking and retains the newest frame. Only its transformation is used. Until a valid frame arrives, the config reference is the start value. With no start value, complete rounds are consumed and dropped until a valid reference arrives. Once a frame reference is received, config references are ignored. Runtime configs replace all filter options; an omitted reference preserves the latched reference. Invalid ranges are ignored with a warning at runtime and rejected at pipeline start.

## Configuration

Configure `initialConfig`, or send a `dai.ImgDetectionsFilterConfig` message to `inputConfig`. All inputs share the same configuration.

| Option | Behavior |
| --- | --- |
| `labelsToKeep`, `labelsToReject` | Optional label ID lists; both apply. An empty keep list removes everything. |
| `setConfidenceRange(min, max)` | Inclusive confidence limits; defaults are 0 and 1 and impose no limit. |
| `setSizeRange(minArea, maxArea)` | Inclusive area limits, in output pixels squared. |
| `setWidthRange(min, max)`, `setHeightRange(min, max)` | Inclusive side limits in output pixels. |
| `regionOfInterest` | Pixel rectangle that must contain all four rotated box corners. |
| `maxDetections` | Keeps the highest confidence detections, preserving their input order. Zero keeps none. |
| `sortByConfidence` | Stable sort, highest confidence first. |
| `overlapMode` | `OFF`, `NMS` (default), or `AVERAGE`; only different input keys can be duplicates. |
| `overlapIouThreshold` | Rotated IoU must be strictly greater than this value; default 0.4. |
| `reference` | Optional start/reference transformation. |

Every range must satisfy minimum < maximum. Setters assign values without validation. The node validates at start and when it receives the last queued runtime config. Other config values are not range-checked.

Processing order is labels, confidence, remap, source visibility, duplicates, geometry, maximum count, sort, then mask construction. Duplicate groups begin with the detection having the largest visible box fraction when source masks are linked, with confidence breaking ties; otherwise they begin with the highest confidence detection. The leader collects at most one same-label detection from each other input, choosing highest IoU, then confidence, then input order. `NMS` keeps the leader unchanged. `AVERAGE` weights box coordinates by confidence multiplied by the visible box fraction (or confidence alone without source masks), retains the leader's label name/keypoints/confidence, and unions member masks. All-zero confidences use visible box fractions as weights, or equal weights without source masks. Equivalent rectangle representations are aligned before averaging angles.

Output order is lexicographic key order followed by each input's detection order; `cam10` precedes `cam2`. A merged box occupies its leader's position. Equal confidences preserve this order. Output sequence number and all timestamps come from the input with the newest host timestamp; earlier key wins ties. Devices have independent sequence counters, so use host timestamps when matching output to other messages.

## Geometry and masks

Remapping uses intrinsics, distortion, and relative camera rotation. It does not use camera translation or object depth. For Cylindrical and Equirectangular references, the filter remaps 16 samples along each box edge before fitting an enclosing rotated rectangle, accounting for edges that curve beyond their projected corners. Other references use the four corners. Output boxes use standard form: -45 < angle <= 45 degrees. Width is the side nearer the horizontal axis. Geometry filters use this form even in pure filtering, where stored box values are preserved. Rotations and overlap are measured in pixels, including for nonsquare images. Partly outside boxes retain true coordinates; boxes with no shared image area are removed.

Unmarked detection coordinates are normalized, including values outside [0,1]. Explicitly mark `RotatedRect` coordinates as pixels to use pixel input. Keypoints use normalized coordinates, as in the existing `ImgDetections` message contract. Remapped keypoints are normalized. Unprojectable keypoints retain their slot with confidence zero.

The output supports Perspective (including distortion), Equirectangular, and Cylindrical references. Version 1 accepts panorama input models only when their transformation equals the reference; other panorama-input remaps raise an exception. Input transformations must be valid in remap mode, even for empty messages. Pure filtering needs a valid transformation only for pixel geometry limits. Incompatible coordinate systems raise the original remap exception. Apply cross-device calibration before starting the pipeline.

For calibrated panoramas, link each `Stitching.outSourceMasks["inputN"]` to `filter.inputSourceMasks[key]` for that camera's detection input. The stitcher emits GRAY8 masks once when the composition is prepared, and again after a reset. Nonzero pixels mark the source's seam region when blending, or the pixels retained after later inputs overwrite it when copying directly. This avoids selecting a high-confidence box from a camera whose object pixels are hidden by another view. Multiband blending can still mix sources near seams.

Source masks are optional and require host execution. The filter latches the latest mask per linked key and drops rounds until every linked mask matches the output transformation. Mask dimensions must match the reference, with a complete GRAY8 payload. Visibility is the number of nonzero mask pixels whose centers lie inside the rotated box, divided by the full box area. The scan is clipped to the output dimensions; portions outside the image contribute no visibility. A box is rejected only when no pixel centers in its footprint are visible, so seam-crossing objects survive even when their box centers are hidden or outside the image. Duplicate leaders prefer the greatest visible fraction, then confidence. Averaging multiplies confidence weights by visibility; all-zero confidences use visibility alone. Maximum-count selection and output sorting still use confidence. With duplicate removal OFF, every partially visible box remains. Unlinked keys retain the original behavior. This selection does not correct parallax in the stitched pixels or align boxes from different camera centers by depth.

A segmentation mask is present when any input has one. In pure filtering it retains its input dimensions; in remap mode it uses reference dimensions. Each output pixel samples the input mask at the inverse-projected pixel center, without interpolation. Mask values are final output indices; removed/invalid indices become 255. NMS keeps only the winner's own mask. Average groups use the union. Among final detections covering a pixel, highest confidence wins; earlier output order breaks ties. Detections at positions 255 and above remain in the list but cannot own mask pixels, and the node logs a warning.

## Execution and panorama example

One input runs on RVC4 by default, and `setRunOnHost(true)` moves it to the host. RVC2, device-free pipelines, and multiple-input filters run on the host. Linked source masks also select host execution. Forcing device placement with multiple inputs or source masks is rejected. Device execution requires RVC4 firmware built with this public node and config. A reference frame linked from a host node to a device filter transfers the entire frame although only its transformation is used; a config reference avoids that transfer.

First create a cross-device calibration JSON with the [MultiDeviceCalibration example](../examples/python/MultiDeviceCalibration/multi_device_calibration.py). Keep the camera rig fixed after calibration. Then run:

```sh
python3 examples/python/MultiDevice/multi_device_detections_panorama.py \
    multi_device_calibration.json yolov6-nano DEVICE_1 DEVICE_2 \
    --projection Cylindrical --average
```

The [C++ example](../examples/cpp/MultiDevice/multi_device_detections_panorama.cpp) uses the same positional arguments and options. Both use CAM_A on each device, calibrate all transformations into a common origin, request undistorted views for stitching, run the same label model per camera, synchronize detections before filtering, and synchronize the panorama and merged detections by host timestamp for display. Omit `--average` to use NMS. Both examples link source visibility masks so duplicate selection follows the panorama seam. The example supports all three reference projections. All devices must be covered by the calibration graph, and the selected model must support each device platform.

With the panorama window focused, press `1` to turn duplicate removal off, `2` for NMS, or `3` to average duplicate detections. `W` increases the IoU threshold by 0.05 and `S` decreases it, clamped to [0, 1]; lowercase keys also work. The initial threshold is 0.40. Higher thresholds require more overlap before detections count as duplicates. The current settings and controls appear on the panorama. Changes are sent through `inputConfig` while the pipeline runs; the confidence threshold remains 0.5. Press `Q` to quit.

Both examples default to `--panorama-scale 2`, stitching 1280 × 800 camera views. Choose a scale from 1 to 4 to multiply the original 640 × 400 view size; for example, `--panorama-scale 3` uses 1920 × 1200 views. The final panorama dimensions depend on calibration and projection. The maximum allowed canvas scales with the views (3200 × 1600 at the default scale); this cap rejects oversized panoramas rather than resizing them. Larger views increase transfer and stitching work without changing the model's inference resolution.

The examples transfer views as NV12, halving image bandwidth compared with BGR. Two default-size views at 30 FPS require about 92 MB/s before protocol overhead; more cameras or larger scales can exceed a shared Gigabit Ethernet link. Both examples show one resizable window with camera previews stacked on the left and the panorama on the right. Previews reuse these views and remap detections into their undistorted coordinates, avoiding additional image transfers. Each panel keeps only the latest frame so a slow UI cannot build a backlog.

The default compositor uses seam estimation, exposure compensation and multiband blending for smoother seams. Add `--no-blend` for faster direct copying of the calibrated, warped views; later cameras replace earlier ones in overlap regions, so seams and exposure differences may be visible. `--blend` explicitly enables the default behavior. Calibration maps are cached in both modes.

The cameras run at 30 FPS. Free-running cameras can have a steady capture offset even with valid geometric calibration. The example accepts a timestamp spread below 33 ms by default; `--sync-threshold-ms` changes the tolerance for detection grouping, stitching and display. Values must be positive and no greater than one frame period (33.3 ms). An overly tight tolerance can repeatedly discard frames and stall the panorama. A wider tolerance pairs frames taken at different moments and can misalign moving objects. For tighter timing, synchronize camera capture using PTP or wired FSYNC as shown in the [frame synchronization example](../examples/python/MultiDevice/multi_device_frame_sync.py), then reduce the tolerance.

Alignment follows the stitched image when `Stitching` uses `PANORAMA` and `setUseInputCalibration(true)`. Known limits are parallax for nearby objects, visual feature-estimated panoramas, planar projections (whose plane is absent from reference metadata), full-360-degree boxes crossing the panorama join, and curved outlines approximated by sampled enclosing rectangles. Label IDs must agree between models, and synchronization is the caller's responsibility.
