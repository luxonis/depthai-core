# PC-launched VIO running in RVC4 firmware

Run this example on your PC. It creates stereo cameras, IMU, Sync, and the new
`dai.node.VIO` node on the OAK-4. Basalt optical flow and visual-inertial estimation
execute on the camera's ARM CPU. Only poses are returned to the PC, where they
are displayed and saved to CSV. No images, depth maps, or IMU samples are streamed
to the PC by this example.

This replaces the earlier standalone-on-camera example. It uses `VIO`, a device
node; the existing `BasaltVIO` host node remains available separately.

## Build and select matching firmware

Both the core SDK and RVC4 firmware need these changes. An unmodified released
firmware does not recognize the new `VIO` node. There is no fallback to PC VIO.

1. Include the core changes in the firmware's `external/depthai-core` checkout.
   During development both working trees must contain the matching API and shared
   Basalt utility files and dependency port patch. A submodule commit bump alone does not include uncommitted
   files. The changes have been applied to both local core checkouts for this task.
2. In the RVC4 firmware repository, enter your normal RVC4 development shell
   (`./start_dev_shell.sh rvc4`) and reconfigure the existing build to install the
   new `vio` vcpkg feature (Basalt). Then build the firmware using your normal
   configuration, for example inside that shell:

   ```bash
   cmake -S . -B build_docker_arm64_rvc4
   cmake --build build_docker_arm64_rvc4 --target depthai-device-rvc4 --config RelWithDebInfo --parallel 4
   ```

   Firmware CMake enables the Basalt dependency for RVC4 and registers the VIO
   node. The Basalt port includes end-of-stream and unstarted-thread cleanup fixes
   needed for safe pipeline shutdown. It does not require enabling the separate host `BasaltVIO` SDK feature.
   The firmware's `vio` feature declares OpenCV directly with defaults disabled,
   selecting only `calib3d` and `highgui` for Basalt. This prevents vcpkg from
   bringing in GTK/gettext and DNN tools through transitive default features.
   `highgui` is compiled without a desktop backend; VIO keeps its GUI disabled.

3. On your **PC**, build/install the updated Python SDK from the core repository:

   ```bash
   python3 -m venv .venv-vio
   source .venv-vio/bin/activate
   python3 -m pip install --upgrade pip
   python3 -m pip install -v ./bindings/python
   ```

   Use the repository's normal compiler and system build prerequisites. The PC
   SDK needs neither an ARM64 target nor `DEPTHAI_BASALT_SUPPORT=ON` for this example.
4. On the PC, point DepthAI at the firmware artifact from step 2:

   ```bash
   export DEPTHAI_DEVICE_RVC4_FWP=/absolute/path/to/depthai-device/build_docker_arm64_rvc4/RelWithDebInfo/depthai-device-rvc4-fwp.tar.xz
   python3 examples/python/RVC4/VSLAM/vio_pose_logger.py poses.csv
   ```

   The firmware archive must be accessible on the PC running the example. If
   needed, select a camera using `DEPTHAI_DEVICE_NAME_LIST=CAMERA_IP`. Stop any
   existing pipeline that is using that camera before starting this one.

For an optional 3D trajectory and camera-pose view:

```bash
python3 -m pip install rerun-sdk
python3 examples/python/RVC4/VSLAM/vio_pose_logger.py poses.csv --visualize
```

The [C++ example](../../../cpp/RVC4/VSLAM/vio_pose_logger.cpp) displays the same
pose values in the terminal and logs the same CSV fields. Build `vio_pose_logger`
with `DEPTHAI_BUILD_EXAMPLES=ON` on the PC, then run
`build/examples/cpp/RVC4/VSLAM/vio_pose_logger poses.csv` with the same firmware
override. Its target has no host Basalt dependency.

## Initial hardware test

Use a stereo OAK-4 with CAM_B/CAM_C and a working IMU. It needs camera intrinsics,
stereo calibration, and camera-to-IMU extrinsics. Nominal IMU translation is used
by default; `vio.setUseSpecTranslation(False)` selects calibrated translation.
This setting does not validate calibration quality or camera/IMU time alignment.

The example validates the CAM_B/CAM_C-to-IMU transforms before starting cameras.
On supported OAK-4 models, firmware fills missing IMU extrinsics from its existing
board design defaults, including when `BoardConfig.defaultImuExtr` is empty.
These defaults are nominal geometry, not a measured camera/IMU calibration.
A device reporting `imuExtrinsics.toCameraSocket = AUTO` has no usable reference;
rebuild/select firmware with this fix. Boards without known defaults need valid
camera-to-IMU calibration. Runtime defaults do not write the EEPROM.


1. Point the camera at a well-lit textured scene; hold it still initially.
2. Check that the terminal prints positions, orientations, and pose age, and that
   `poses.csv` gains rows. Move/rotate the camera slowly and observe the response.
3. Move through a measured distance and back to assess scale, drift, and latency.
4. Stop and restart the example, then test dropouts and the real vehicle workload.

Input is 640x400 unrectified GRAY8 stereo. The Python example requests 90 FPS,
caps it to the highest rate supported by both cameras' advertised modes,
and prints the selected rate before startup. HDR modes are excluded when the SDK
exposes that flag. The C++ example requests 30 FPS.
Both configure raw accelerometer/gyroscope at 200 Hz.
These are input settings, not a verified pose rate. Firmware uses
Basalt's real-time mode; estimator frames and the PC's bounded output queue can
drop samples under load. Reduce camera FPS if firmware reports queue overflow.
RVC4 IMU startup can take several seconds longer than camera startup. Firmware
consumes and discards stereo frames until valid paired IMU samples arrive, then
starts Basalt on fresh images. It allows up to 30 seconds for sensor startup;
startup frames do not count toward the 64-frame pending-pose limit.
Input queues and retained source-frame metadata are bounded. Ctrl+C stops the pipeline.
The CSV file is overwritten on each run; use a new filename per session.

## CSV and integration with LiDAR/GNSS

| Columns | Meaning |
| --- | --- |
| `sequence` | Source left-camera frame sequence number; gaps are possible. |
| `sample_time_s` | Source frame timestamp synchronized to the PC's `dai.Clock` monotonic clock. |
| `device_time_s` | Same frame's device monotonic timestamp, used by the estimator. |
| `system_time_s` | Device system-clock timestamp in Unix seconds, if available; empty otherwise. |
| `age_s` | PC receive time minus `sample_time_s`, including processing/transport/queue delay. |
| `x_m,y_m,z_m` | Left-camera position in the local VIO world, in metres. |
| `qx,qy,qz,qw` | Left-camera FLU orientation in that world; quaternion is scalar-last. |

FLU means forward, left, up. The world is local, not georeferenced; its heading
is not true north. Each pipeline restart creates a new odometry session. The
firmware matches each estimator state to its source frame and preserves all
three timestamp domains; receipt time is not substituted for measurement time.

To compare VIO with LiDAR/GNSS, calibrate sensor-to-vehicle transforms including
lever arms, synchronize the clocks, and align the local navigation frames.
Device system timestamps alone do not establish PTP/GNSS synchronization. Compare
relative motion over short windows with uncertainty and persistence thresholds;
combine discrepancies with GNSS fix quality and LiDAR registration diagnostics.
Disagreement alone does not identify which sensor failed.

The logger detects stale/missing output and non-finite poses. These are liveness
checks, not tracking-confidence checks. This pose API supplies no covariance or
tracking-quality flag. VIO can drift or continue producing inaccurate poses in
low texture, darkness, glare, vibration, or rapid motion.

## Validation

After building the core tests, with the custom firmware selected, run:

```bash
ctest --test-dir build -R '^vio_(properties|device)_test$' --output-on-failure
```

The device test checks OAK4-D IMU defaults with an empty board config, pose
finiteness, exact source timestamps/sequence numbers,
stopping without sensor input or after more than 64 frames without IMU,
pose output after a deliberate five-second IMU delay, and
stopping/restarting after pose output. It
requires the scene/calibration described above and also exports images for
metadata comparison; the normal example exports only poses.

The updated sources and regression tests still need to be built and run. A sensor
timing diagnostic using the previously built firmware on OAK4-D / OAK OS 1.37.0
measured stereo startup at 0.37 seconds and paired IMU startup at 4.23 seconds,
confirming why the previous 64-frame limit fired before any pose could be produced.
A separate diagnostic held back stereo until IMU was ready using the existing
firmware; on-device VIO then returned 323 finite poses with increasing timestamps
in a 15-second run. This confirms the startup-order diagnosis, but does not replace
running the regression tests against the rebuilt firmware.
Validate sustained operation on an OAK-4 before using this as a customer deployment.
