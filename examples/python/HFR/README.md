# HFR (High Frame Rate) examples on IMX586 on the RVC4 platform
This directory contains examples demonstrating high frame rate (HFR) capabilities on the IMX586 sensor using the RVC4 platform.
The examples showcase an early preview of the capabilities.


## HFR resolutions
The HFR mode introduces two new resolutions that can run at a high frame-rate.
At the current time, the resolutions cannot be scaled arbitrarily nor can the FPS be varied arbitrarily — it must be one of the two supported values.
We plan to add more flexibility in the future and it's possible to use `ImageManip` in the meantime.

The current supported resolutions are:
* 1920x1080 @ 240 FPS
* 1280x720 @ 480 FPS


## Example descriptions
### Object Detection
The object detection [example](hfr_nn.py) demonstrates how to use the HFR mode with a YOLOv6 model for real-time object detection at **480 FPS**.

### Small live preview
The small preview [example](hfr_small_preview.py) demonstrates how to use the HFR mode for a small live preview at **240 FPS** or **480 FPS**.

### Video encoding
The video encoding [example](hfr_save_encoded.py) demonstrates how to use the HFR mode for video encoding at **240 FPS** or **480 FPS**.

All three examples have a `BenchmarkIn` node included that prints the framerate and the latency. This is the expected output:
```
[2025-08-14 23:31:49.487] [ThreadedNode] [warning] FPS: 474.3766
[2025-08-14 23:31:49.487] [ThreadedNode] [warning] Messages took 1.0118543 s
[2025-08-14 23:31:49.487] [ThreadedNode] [warning] Average latency: 0.05904912 s
```
