# Record node catalog notes

Date: 2026-11-06. Host binary: `build/tests/record_node_host_test`. Full compact run: 99 cases, 90 passed, 8 failed, 1 skipped, 1938 assertions (1930 passed). RNG seed `3719039882`. Log: `/tmp/record_host_run.log`. Expected catalog values were not changed.

`ffmpeg` 9.0. `cv2` 4.10.0. `clang-format-18` is not on this machine. The local generator is Unix Makefiles. Device and perf binaries were built with `cmake --build ... --target record_node_catalog_device_test record_node_perf_test`.

## Cases not written

Every catalog id in sections 9 to 11 has one `TEST_CASE`. Section 12 gaps were left as gaps.

The stub RED baseline in section 14 item 2 was not produced. `RecordVideo` already has the real API, so the suite was compiled and run against that implementation.

Device catalogs (`VR1`–`VR26`, `VV1`–`VV5`, `VH1`–`VH13`) and perf cases `VD1`–`VD4` compile. They were not run on an OAK4. `VD1`–`VD3` have no measured rates or frame timings. `H1` did not get a 1 MiB tmpfs: the process is not root, and the case skipped with `needs root for a 1 MiB tmpfs`.

Python `PY1`–`PY8` are in `bindings/python/tests/test_record_node.py`. `PY7` and `PY8` are marked `rvc4` and were not run on a device. CMake was not reconfigured after the Python files were added; pytest was invoked directly.

## Expected values

No catalog value is believed to be wrong. The eight host failures below are implementation results against the written expectations. A second oracle was not needed to change a number.

## RED baseline

Not run. See above.

## Host failures that stay in the suite

| Case | Result |
| --- | --- |
| A12 | `setRecordingName("")` then `start()` throws nothing. `Record.cpp` replaces an unset or empty name with `makeDefaultRecordingName()` before `validateRecordingName`. |
| A20 | Recording directory has 4 entries. The two stream directories are present, and so are `.dai_closed_left` and `.dai_closed_right`. |
| R5 | ERROR for sequence `-1` is reported and the text contains `-1`. `allFiles(recDir)` is not empty. The extra name is `.dai_closed_cam`. |
| V1, V2, V6, V7 | `expectFiles` fails because `.dai_closed_enc` is in the directory. V2's printed set is `.dai_closed_enc`, `enc.mcap`, `enc.mp4`, `enc_s1.mcap`, `enc_s1.mp4`. V6's printed set is `.dai_closed_enc`, `enc.avi`, `enc.mcap`. |
| V3 | HEVC in VIDEO format reaches the empty-directory check, which fails because `.dai_closed_enc` remains. |
| H1 | Skipped. The 1 MiB file system was not available. |

Isolated reruns: A1, A2, and A3 pass alone. A12 fails alone. The line-31 failure in the full log is `requireStartError` inside A12.

## Measured VD values

None. `record_node_perf_test` links and was not executed. `VD4` is in that binary as well; section 14 asks for `VD1`–`VD3`.

## Gates

Not run. The host suite is not all green, so the section 14 merge gate is not met. That includes three `--order rand` runs, a load run, `ctest --repeat until-fail:20` on groups T and P, and the mutation spot-check of T2, P1, P10, K1, and V2.

## Python

`PY3`, `PY4`, and `PY6` passed. `PY6` calls `setStreamFps(0.0)`.

`PY1`, `PY2`, and `PY5` originally read the status queue after `pipeline.stop()`, which closes it. They now call `record.getStatus()` through `status_after_stop()`. Expected STOPPED tuples are unchanged. After that change, `PY1` and `PY5` printed `1 passed` and then the interpreter exited 132 during finalization (`_Py_Finalize` / `backward::SignalHandling`). `PY2` exited 0. Those exit codes are shutdown crashes after a passing body.

## Harness choices

- Spec setter names are wrapped in `record_test`. `setFps(float)` calls `setStreamFps`. `setFps(unsigned)` calls `setFps`.
- `inputControl.createInputQueue()` is created before `pipeline.start()` in cases that send `RecordControl`. It is not created in the `HostRecord` constructor. A16 and P7b do not create a control queue.
- `MessageQueue::get<T>(duration)` is not the public timeout form. `record_test::getFor` uses `get<T>(duration, bool&)`.
- `mcap::McapReader::readSummary` uses `AllowFallbackScan`.
- Node logs are stdout spdlog lines. `LogCapture` tees stdout and `watchNode` sets the node log level to DEBUG. `DeviceLogCapture` does the same for device lines. Both have `texts()` and `textsAt(LineLevel)`.
- `RecordingGuard` snapshots names at construction. `retarget` and `release` exist so a second pipeline can keep the recording under test. `Store::replacePipeline()` retargets the same guard. `restart()` still deletes new recordings.
- CMake enables the catalog when `DEPTHAI_HAVE_OPENCV_SUPPORT` and `DEPTHAI_ENABLE_PROTOBUF` are both on. The spec text says `DEPTHAI_OPENCV_SUPPORT`.
- Device catalog binary is `record_node_catalog_device_test` so it does not collide with the older `record_node_device_test`. Labels are `ondevice`, `rvc4`, `ci`, `noreplayci`. The CMake label `replay` is not used. `RESOURCE_LOCK device` is set on the catalog and perf tests.
- `T(W×H)` intrinsics use `fy = W/2` (`record_test::intrinsicsT`). `FrameEvent` is not in the ImgFrame serialize list, so a loaded frame has `NONE`. The `[data]` self-check covers that.
- Catch2 rejects `operator||` inside `REQUIRE` / `CHECK`. VR21 assigns the bool first.
- `VR1` still requires `sizeBytes` to equal the sum of the five `.dai` files.
