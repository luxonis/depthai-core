# Performance and resources

Use for low FPS, growing latency, drops, memory growth, thermal or power symptoms.
Compare the same device, inputs, firmware, OS, settings and measurement window.
Record warm-up, duration and frame count. Keep a normal-logging baseline.

## Measure the limiting stage

| Measurement | What to retain / distinguish |
| --- | --- |
| Source | Requested/effective FPS, exposure, resolution and sequence progression |
| Node | Valid pipeline state timings, custom stage timings, Benchmark reports |
| Link | Negotiated USB/network speed, bytes/sec and payload size |
| Host consumer | Arrivals, sequence gaps, processing/display/sink time |
| Latency | Capture/device/host timestamps with known clock domains; distribution and growth |
| Resources | CPU, memory, pool/queue occupancy, temperatures, operating clocks/profile |

Use `SystemLogger` with a drained, bounded output queue. RVC2 produces
`SystemInformation` (including LEON/CMX/DDR metrics); RVC4 produces
`SystemInformationRVC4` (including process/system CPU and memory). Check field
units in headers; identically printed units do not imply identical underlying
representations. Direct Device resource getters are platform-dependent.

Use the existing BenchmarkIn/BenchmarkOut examples to count a device-side rate,
then count what the host receives. A fast internal counter does not prove a fast
consumer. For data-transfer counters:

```bash
DEPTHAI_LEVEL=debug DEPTHAI_PROFILING=1 python3 -u /path/to/repro.py
```

This profiles transfer, not the execution time of every node. Trace logging,
GUI rendering, saving images and event subscriptions can become the bottleneck.
Compare instrumented and ordinary runs.

If the producer never exceeds the measured rate, the experiment cannot establish
the downstream ceiling. Increasing source load is an active comparison; retain
the original settings and watch latency and drops as well as throughput.

## Memory and scheduling

Measure trends over equal-duration runs. Separate RSS from live allocations,
device pools from host queues, and allocator retention from leaks. Retained
frames can exhaust pools outside visible queues. Check payload size, number of
buffers and fan-out. Do not universally reduce pools or make queues nonblocking:
either can change behavior. Vary one suspect queue/pool while measuring results.

Use host/thread stacks, supported CPU/allocation profilers, or user-built
sanitizer variants when a resource trend needs attribution. On RVC4 include
kernel OOM evidence, tmpfs/core-dump space, system versus process usage, thermal
throttling and effective power profile. CPU utilization alone does not measure
GPU/DSP saturation.

## Deterministic and live experiments

Use record/replay or injected frames to separate application/algorithm behavior
from live acquisition. Verify input/passthrough bytes and metadata before drawing
a comparison. SDK holistic controls are:

```bash
DEPTHAI_RECORD=/path/to/new-recording python3 -u /path/to/repro.py
DEPTHAI_REPLAY=/path/to/recording python3 -u /path/to/repro.py
```

Check the selected revision's supported topology and recording format.
Recording itself can affect timing and stores potentially private imagery.
Replay does not prove equivalence for sensor, driver, bus, power, synchronization
or real-time behavior. If replay passes and hardware fails, keep those hypotheses.

## Active health check

This acquires a device and can start streams, measure bandwidth, and exercise IR
power load. Run only on an explicitly selected available target within the
authorized test scope. Inspect `HealthCheckConfig` and choose needed checks;
defaults enable more than a passive status query.

```python
# Select a target; never choose an arbitrary first device on shared hardware.
info = dai.DeviceInfo(selected_device)
config = dai.HealthCheckConfig(
    measureBandwidth=False,
    verifyCameraFunctionality=False,
    verifyImuFunctionality=False,
    verifyPowerSupply=False,
)
metrics = dai.Device.performHealthCheck(info, config)
print(metrics)
```

The remaining defaults still perform other checks and open a connection.
`NOT_RUN` is not a pass. A power-load failure establishes behavior under that
load, not automatically a software defect or defective power supply.
For load testing, record duration, cameras/IR used and restore altered state.

Sources: [SystemLogger example](../../../../examples/python/Misc/SystemLogger/system_logger.py),
[benchmarks](../../../../examples/python/Benchmark),
[record/replay examples](../../../../examples/python/RecordReplay),
[health configuration](../../../../include/depthai/device/HealthCheck.hpp),
[health implementation](../../../../src/device/HealthCheck.cpp),
[profiling API](../../../../include/depthai/utility/ProfilingData.hpp).
