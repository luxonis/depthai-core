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

At 1 Hz, mean processor power was 1.105 W versus 1.119 W in adjacent raw samples; PoE was 1.505 W versus 1.529 W. These are independently timed readings, not atomic equality checks. GPU/HVX/HMX/Q6 all reported zero in this idle run. The absent system rail remained unavailable; the ToF rail was a valid zero. Other boards, RVC2 hardware and other desktop platforms remain unvalidated; the workload results below extend the initial idle-only check.

The broader existing serialization regression has an unrelated RVC4 failure: its Roundtrip case calls unsupported StereoDepth.setOutputSize. The new message-specific tests pass. Firmware artifact publication is required before the pinned SDK can build without a local firmware override.

Final paired-build validation: SDK library, Python module, C++ example and firmware package/license generation completed. The Python binding regression module passed all 16 tests (run with `--noconftest` because the repository's unrelated legacy test-support module is unavailable). Both mirrored examples emitted the new metrics on hardware. The bounded Python runner passed at requested 1 Hz and 10 Hz. All test processes disabled telemetry and automatic calibration.

## GPU workload validation

A compute-only OpenCL workload was run on an Adreno 740 after the initially idle camera was checked for other users. It selected CL_DEVICE_TYPE_GPU, profiled command execution and validated output readback. No camera configuration or shared counters were changed.

The sustained phase completed 191 kernels, with 19.859 seconds of profiled GPU execution over 20.083 seconds. After a fixed three-second settling exclusion, SystemLogger averaged 98.36% (33 samples) and direct vendor reads averaged 98.37% (168 samples). All 33 nearest sample pairs matched exactly. Idle-before and recovery phases returned zero; across all four stable phases, all 96 nearest pairs matched (maximum receipt-time separation 49.75 ms). Independent reads are not atomic, and this does not calibrate physical occupancy.

The intermittent phase completed 45 kernels with 4.679 seconds of profiled execution over 20.329 seconds (about 23% of wall time). Yet the stable vendor sample means were only 2.97% from SystemLogger and 4.01% from direct reads, with many zeros between bursts. This is an inherited vendor-counter limitation: do not interpret the field as a duty-cycle average over the logger interval. A zero vendor reading does not prove that no GPU work occurred during that interval.

## DSP and neural inference workload validation

On the same OAK4, camera-free synthetic inputs exercised EVA stereo with DSP median/spatial postprocessing (611 received disparity outputs) and NEURAL_DEPTH_480X300 with explicit DSP runtime (767 received inference outputs). Each run included idle, intermittent input, sustained input and recovery. SystemLogger ran at 2 Hz; independent vendor reads targeted 50 Hz. The first three seconds of each phase were excluded from comparisons; pairs were selected by nearest host receipt timestamp, not value.

During sustained neural inference, 42 SystemLogger samples versus 1,037 direct reads averaged HVX 64.05% versus 64.00%, HMX 48.00% versus 48.04%, and Q6 260,181.57 versus 256,226.17 KCPS. Exact nearest-pair counts were 40/42, 37/42 and 35/42 respectively; mean absolute differences were 0.286 percentage points, 0.214 percentage points and 866.10 KCPS. Maximum receipt separation was 12.06 ms. Independent reads are not atomic, so this supports vendor-counter tracking, not guaranteed equality for every sample or calibrated NPU occupancy.

Both sources returned zero in the stable idle and recovery windows. DSP postprocessing produced outputs but all three metrics stayed zero in stable windows; this workload alone does not validate nonzero activity. Intermittent neural inference produced HMX/Q6 bursts while HVX remained zero. A zero vendor reading cannot establish absence of work. None of these readings measures awake residency or aggregate NPU utilization. The extra 50 Hz raw sampler's overhead was not isolated; the earlier collector timing is a separate measurement. No sensor settings or shared counters were changed.
