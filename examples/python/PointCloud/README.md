# PointCloud examples (Python)

| Script | Shows |
| --- | --- |
| `point_cloud.py` | The minimal flow: stereo depth into one `PointCloud` node, points printed on the host. |
| `point_cloud_showcase.py` | Length unit, organized output, target coordinate systems and custom transforms, one `PointCloud` node per feature. |
| `point_cloud_visualizer.py` | A colorized cloud in the DepthAI visualizer. |
| `multi_device_point_cloud.py` | One merged cloud from the depth of several devices (below). |

The C++ counterparts live in `examples/cpp/PointCloud/`; its `README.md` documents the node itself
(coordinate-system transforms, the rules of a merged cloud, the compute methods).

## multi_device_point_cloud.py

One `PointCloud` node fed by every device of the pipeline. Each device computes depth aligned to its
color camera and both streams are linked to the node under the device ID:

```python
depth.depth.link(pc.getDepthInput(deviceId))
colorOut.link(pc.getColorInput(deviceId))
```

The node synchronizes the streams (`pc.sync`, threshold set to one frame period because the devices are
not hardware-synchronized), deprojects each with its own intrinsics and merges them into a single
colorized `PointCloudData` expressed in the common origin of the multi-device calibration. The merged
cloud is published to the DepthAI visualizer (http://localhost:8082, press Q to quit) and its extent is
printed.

### Requirements

- At least two devices (the first two available ones unless `--devices` is given).
- A multi-device calibration JSON describing where the devices stand relative to each other. Create it
  with `../MultiDeviceCalibration/multi_device_calibration.py`; the default path is
  `multi_device_calibration.json` next to this script.
- A `depthai` build with beta support (`DEPTHAI_BUILD_BETA`, on by default) for
  `dai.beta.MultiDeviceCalibrationHandler`.

### Running

```bash
python3 multi_device_point_cloud.py                                 # first two devices, calibration next to the script
python3 multi_device_point_cloud.py -d 10.0.0.11 10.0.0.12 -c rig.json
python3 multi_device_point_cloud.py --size 320 200                  # lighter streams when many devices share one link
python3 multi_device_point_cloud.py --target-device <id> --target-socket CAM_A
python3 multi_device_point_cloud.py --target-device <id> --target-housing VESA_A
```

| Option | Meaning |
| --- | --- |
| `-d`, `--devices` | Device IDs or IPs (default: the first two available devices). |
| `-c`, `--calibration` | Multi-device calibration JSON (default: `multi_device_calibration.json` next to the script). |
| `--size W H` | Depth and color size per device (default: 640 400). |
| `--target-device` | Express the merged cloud in a coordinate system of this device instead of the common origin. |
| `--target-socket` | Camera socket of the target device, for example `CAM_A` (default). |
| `--target-housing` | Housing coordinate system of the target device instead of a socket, for example `VESA_A`. |

With `--target-device` the frames of the other devices are carried over through the multi-device
calibration and that device's own calibration, so the cloud can be expressed in any camera or housing
of any device in the pipeline.

### Compute backend

PointCloud and ImageManip share `dai.Backend` (`CPU`, `HW`, `GPU`, `AUTO`).
PointCloud supports CPU, GPU and AUTO; HW raises an error. CPU with one thread
per stream is the default. AUTO resolves to GPU with CPU fallback; `getBackend()` reports GPU.

```python
pc.setBackend(dai.Backend.GPU)
pc.setBackend(dai.Backend.CPU).setNumThreads(4)
```

The existing `useCPU()`, `useCPUMT(n)` and `useGPU(device)` methods remain available.
`PointCloudProperties.ComputeMethod` and the `computeMethod`, `numThreads` and `gpuDevice`
properties keep their existing meanings, defaults and serialized layout. The new methods adapt
that same configuration. GPU fallback remains single-threaded.
The platform GPU implementation is selected internally.
