# Focused depth on RVC4

Use Python bindings built from this branch. From the repository root, run one mode at a time:

```sh
python examples/python/Depth/focused_depth.py --mode roi --roi 0.35,0.30,0.65,0.70 --fps 30
python examples/python/Depth/focused_depth.py --mode detector --confidence 0.25 --fps 30
```

Open the Visualizer URL printed by the command. Press Ctrl+C to stop. Use
`--webSocketPort 8767 --httpPort 8084` if the default ports are occupied.

The ROI coordinates refer to the rectified left camera. Detector mode selects
only the largest bounding box after mapping detections to that camera. Color
and detection views use the color-camera coordinates; the focused depth view
uses the left-camera coordinates. Pixels outside the selected box are zero.

The default neural model is **192x120 per crop**, reassembled into the full-size
left image. `--depth-model` selects a larger model at a potential throughput cost.
The single-model, largest-region path overlaps three frames; this adds two frames
of buffering. It is intended for continuous camera streams, not finite batches.

`--fps` requests the camera rate. Actual depth FPS and the number of nonempty
frames are reported after five seconds of warm-up. To measure without rendering:

```sh
python examples/python/Depth/focused_depth.py --mode roi --fps 30 --headless --seconds 30
python examples/python/Depth/focused_depth.py --mode detector --confidence 0.25 --fps 30 --headless --seconds 30
```

No detection produces an empty depth frame. The `--confidence` threshold may need
adjustment for the scene. The ROI and detection wrapper scripts call the same
implementation. The budget wrapper is best effort and cannot guarantee an FPS.
