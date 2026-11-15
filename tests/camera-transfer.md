# Camera transfer qualification

`camera_transfer_test` receives live IMX378 frames from CAM_A through the
ordinary Camera output queue. Its default qualification requires USB 3,
4032 × 3024 NV12, and at least 3 Gbit/s of received pixel payload over 60
seconds after a five-second warm-up. The sensor runs at 30 FPS. The first
frame establishes the measurement boundary and is not counted.

Build the test with Core's other C++ tests. Select the reserved device using
`DEPTHAI_PLATFORM` and `DEPTHAI_PROTOCOL`, with `DEPTHAI_DEVICE_NAME_LIST`
for a USB port path or `DEPTHAI_DEVICE_ID_LIST` for a device ID. Select the
candidate firmware with the platform's normal firmware override.
Run `ctest --test-dir build -R '^camera_transfer_test$' --output-on-failure`.
The test has no CI platform label until the qualifying hardware run passes.

The test checks dimensions, type, exact payload size, increasing sequence
numbers and device timestamps, and a distinct pixel sample hash for each
measured frame. Its reported
sequence gaps include intentional frame-rate reduction; they are not a
count of lost USB transfers. Payload throughput excludes metadata and USB
protocol overhead.

For functional or CPU comparisons, set `DEPTHAI_TRANSFER_FPS`,
`DEPTHAI_TRANSFER_SECONDS` and `DEPTHAI_TRANSFER_MIN_GBPS`. Setting the
minimum to zero removes the throughput threshold while keeping frame
validation. `DEPTHAI_TRANSFER_OUTPUT` saves a CSV and one NV12 frame using
that path as the filename prefix. The default 3 Gbit/s threshold always
requires at least 60 seconds and a SuperSpeed connection.

`profile_camera_transfer.py` runs the same test and samples Linux CPU counters
over its measured interval. It needs a Linux host and SSH access to the
reserved device. For each transport, pass the executable, device name,
protocol, SSH destination, output prefix, FPS and duration through its CLI;
`--help` lists the arguments. Existing DepthAI platform and firmware
environment variables are preserved. SSH uses the normal host-key policy;
`--ssh-option` passes an additional OpenSSH option.

Alternate at least three USB and TCP runs at the same resolution, format
and delivered frame rate. Compare measured payload rates before comparing
CPU. The JSON records raw counters and percentages of one CPU core: Linux
user, system, IRQ, softirq and steal time are busy; idle and I/O wait are
excluded. Firmware process CPU and host application CPU are reported
separately. This measures Linux, not a separate real-time processor. Keep
diagnostic tracing and unrelated device workloads out of the measured run.

`usb_receive_lifetime_test` checks a separate host memory contract. It holds
eight four-megabyte buffers while issuing an RPC, closing the device and
reconnecting, then compares every retained byte with its expected value.
It runs on USB devices and skips other transports. Run it with the host's
default USB memory budget as well as any larger budget used for throughput
measurements.

Record Linux's `usbcore.usbfs_memory_mb` for each run. Mapped receive buffers
share half that budget across connections, with at most 64 MiB per connection.
A frame that does not fit uses ordinary memory. A larger host budget can thus
change the transfer path for high-resolution frames. Restore any temporary
budget increase after measurement. Also compare delivered rates at different
requested camera rates; producing more frames than the link carries can lower
throughput while increasing CPU work.
