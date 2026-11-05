# RVC4 SystemLogger resource metrics

`SystemInformationRVC4` includes optional accelerator and power readings. Each
new reading is a `double`; `-1` means the value is unavailable. A value of zero
is a real zero reading.

| Field | Meaning |
| --- | --- |
| `gpuUtilizationPercent` | Whole-GPU utilization percentage reported by the vendor counter. It is not per-engine utilization. |
| `dspHvxActivityPercent` | DSP HVX activity percentage reported by the vendor counter. It describes activity, not awake residency or total NPU utilization. |
| `dspHmxActivityPercent` | DSP HMX activity percentage reported by the vendor counter. It describes activity, not awake residency or total NPU utilization. |
| `dspQ6LoadKcps` | DSP Q6 load in KCPS reported by the vendor counter. It is not awake residency or total NPU utilization. |
| `powerSystemWatts` | `vdd_5v0` rail reading, converted from microwatts to watts. |
| `powerProcessorWatts` | `vdd_3v9` rail reading, converted from microwatts to watts. |
| `powerPoeWatts` | PoE rail reading, converted from microwatts to watts. |
| `powerTofWatts` | ToF rail reading, converted from microwatts to watts. |

The vendor counter sampling-window duration is not exposed by these fields and
must not be assumed to equal the SystemLogger output interval. The API reports
no accelerator frequency or awake-residency value.

The four power values are separate rail readings. Do not sum them to estimate
total device power.

## Firmware and wire compatibility

`SystemInformationRVC4` uses a positional libnop wire structure. Its decoder
requires the serialized structure to contain exactly the same number of
members as the local definition. Adding fields therefore makes an extended
payload unreadable to an old SDK, and an old payload unreadable to the new SDK.
Use the SDK with matching RVC4 firmware that includes these fields.

The C++ and Python SystemLogger examples print the new metrics and display
`unavailable` for values equal to `-1`.

## Validation and limits

The paired firmware is reviewed in [MR !448](https://gitlab.luxonis.com/luxonis/depthai/depthai-device-kb/-/merge_requests/448).
An idle OAK4 running OS 1.47.0 passed 270 SystemLogger assertions with the paired schema, including existing CPU, memory and temperature fields. The host serialization suite for this message passed 19 assertions; five firmware fixture cases covered missing and malformed sensors, duplicate labels and sensor disappearance/reappearance.

A 100-sample device collector probe averaged 2.00 ms per read (maximum 3.01 ms). This is collector wall time, not whole-pipeline overhead. Twenty messages at a requested 1 Hz averaged 1.022 s between deliveries; twenty at requested 10 Hz averaged 0.120 s (about 8.35 Hz). Existing SystemLogger scheduling sleeps before collecting measurements, so its configured rate is not an exact delivery-rate guarantee.

At 1 Hz, mean processor power was 1.105 W versus 1.119 W in adjacent raw samples; PoE was 1.505 W versus 1.529 W. These are independently timed readings, not atomic equality checks. GPU/HVX/HMX/Q6 all reported zero in this idle run. The absent system rail remained unavailable; the ToF rail was a valid zero. Active DSP workloads, other boards, RVC2 hardware and other desktop platforms remain unvalidated; the GPU workload results below extend the initial idle-only check.

The broader existing serialization regression has an unrelated RVC4 failure: its Roundtrip case calls unsupported StereoDepth.setOutputSize. The new message-specific tests pass. Firmware artifact publication is required before the pinned SDK can build without a local firmware override.

Final paired-build validation: SDK library, Python module, C++ example and firmware package/license generation completed. The Python binding regression module passed all 16 tests (run with `--noconftest` because the repository's unrelated legacy test-support module is unavailable). Both mirrored examples emitted the new metrics on hardware. The bounded Python runner passed at requested 1 Hz and 10 Hz. All test processes disabled telemetry and automatic calibration.

## GPU workload validation

A compute-only OpenCL workload was run on an Adreno 740 after the initially idle camera was checked for other users. It selected CL_DEVICE_TYPE_GPU, profiled command execution and validated output readback. No camera configuration or shared counters were changed.

The sustained phase completed 191 kernels, with 19.859 seconds of profiled GPU execution over 20.083 seconds. After a fixed three-second settling exclusion, SystemLogger averaged 98.36% (33 samples) and direct vendor reads averaged 98.37% (168 samples). All 33 nearest sample pairs matched exactly. Idle-before and recovery phases returned zero; across all four stable phases, all 96 nearest pairs matched (maximum receipt-time separation 49.75 ms). Independent reads are not atomic, and this does not calibrate physical occupancy.

The intermittent phase completed 45 kernels with 4.679 seconds of profiled execution over 20.329 seconds (about 23% of wall time). Yet the stable vendor sample means were only 2.97% from SystemLogger and 4.01% from direct reads, with many zeros between bursts. This is an inherited vendor-counter limitation: do not interpret the field as a duty-cycle average over the logger interval. A zero vendor reading does not prove that no GPU work occurred during that interval.
