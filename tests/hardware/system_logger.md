# SystemLogger counter availability

The regular `system_logger_test` checks metric ranges and permits `-1` on
platforms without the corresponding counters. Availability depends on the
camera hardware and OS image; an RVC4 platform check alone is insufficient.

For a camera/OS with known counters, set the space-separated list of expected
`SystemInformationRVC4` field names. Derive this list from the hardware/firmware
configuration or independent vendor-counter checks, never from the logger
messages under test. Keep it in the testbed configuration when running in CI.
The regular test also enforces this list whenever it is set.

Run the explicit availability test to require a non-empty configuration:

```bash
export DEPTHAI_TELEMETRY=0
export DEPTHAI_AUTOCALIBRATION=OFF
# Example for a configuration exposing all three DSP counters:
export DEPTHAI_TEST_SYSTEM_LOGGER_REQUIRED_METRICS="dspHvxActivityPercent dspHmxActivityPercent dspQ6LoadKcps"
./build/tests/system_logger_test '[systemlogger-availability]'
```

Supported names: `gpuUtilizationPercent`, `dspHvxActivityPercent`,
`dspHmxActivityPercent`, `dspQ6LoadKcps`, `powerSystemWatts`,
`powerProcessorWatts`, `powerPoeWatts`, `powerTofWatts`.
Include every counter expected on the selected testbed. Zero is a valid reading;
no workload is required to test availability. Every configured counter must
produce a non-negative reading within ten messages; initial unavailable samples
are tolerated. Other counters retain sentinel/range checks. Missing or empty
configuration fails the explicit test, unknown names fail both tests, and a
configured availability check cannot run against RVC2.

Record the device identity, OS version, firmware revision and expected-metrics
list with the result. The explicit test is hidden from the generic suite because
that suite also covers devices without these counters.
