# Replay node catalog notes

Date: 2026-11-06. Host binary: `build/tests/replay_node_host_test` (129 `TEST_CASE`s, including the `[data]` self-check). Expected catalog values were not changed.

`ffmpeg` 9.0. `cv2` 4.10.0. `clang-format-18` is not on this machine. The local generator is Unix Makefiles. `replay_node_catalog_device_test` and `replay_node_perf_test` both linked on 2026-11-06.

There is no single Catch summary for all 129 cases. The first full run was killed during E6 teardown. Later runs covered the unmeasured tail. Results below are from the compact logs `/tmp/replay_host_run.log`, `/tmp/replay_host_rest.log`, and `/tmp/replay_tail/`.

## Cases not written

Catalog ids the spec leaves out were not invented: A10–A12, A20, B24, B31–B33, B47, B48, and P5. Section 12 gaps stay gaps.

The stub RED baseline in section 14 item 2 was not produced. `ReplayVideo` already has the real API.

Device catalogs `VA1`–`VA36`, `VB1`–`VB19`, and `VC1`–`VC19`, plus perf `VD1`–`VD4`, compile and were not run on an OAK4. `VD1`–`VD4` have no measured fps, CPU, or timestamp spans. `VD2`–`VD4` skip with `needs the generated data` unless `tests/scripts/make_replay_test_data.py --perf` has been run. That flag was not passed. `plain60_h264.mp4` is generated without `--perf`.

Python `P1`–`P4` and `P6`–`P9` are in `bindings/python/tests/test_replay_node.py`. `P8` and `P9` are marked `rvc4` and were not run. CMake was not reconfigured after the Python file was added.

## Expected values

No catalog value is believed to be wrong.

`4608000` in G3 is `640 * 480 * 1.5 * 10`. The case failed because start did not throw, not because a second size oracle disagreed. `966.7` in E5 failed by 103.3 ms and was left as written. The 60-frame generator wraps Y with `& 255` for frame index `n >= 35`. Thirty-frame catalog values stay in `16..219`. `plain60` is the wrapped source and is documented here rather than edited.

`kHolisticConfigNoStreams` is the section 6.6 holistic JSON with the `streams` key removed. VC6's `recordConfigWithoutStreams` is not pasted in the spec. `VC10` skips with `needs yolov6-nano` when the zoo model cannot be opened.

## RED baseline

Not run. See above.

## Host results

The first compact run printed a `FAILED` line only for A22, B10, B22, B23, B25, B30, E5, and E6, then hung in E6 teardown and was killed. Cases before E6 that printed no `FAILED` line completed. There is no summary line for that prefix.

B23 and B25 failed in that run because `makeDaiFrame` did not create the parent directory (`Failed to open file for writing`). `makeDaiFrame` now calls `create_directories` on the parent. Expected sequence `7` and `pixelsEqual` were not changed. In the later filtered run, B23, B25, E7, and E8 were the four passing cases.

| Case | Result |
| --- | --- |
| A22 | Second `start()` throws `MessageQueue was closed`. |
| B10 | Start throws `ReplayVideo archive entry escapes: /etc/hostname`. The catalog substring is `cam_0.dai`. `ReplayArchive.cpp` puts the symlink target in the message. |
| B22 | Start throws `ReplayVideo duplicate frame number 0 in <dir>`. The text contains neither `cam` nor `dog`. `ReplayDecode.cpp` reports the duplicate sequence before the multi-stream prefix list. |
| B30 | Same duplicate-number message. It contains neither `cam_01.dai` nor `cam_1.dai`. |
| E5 | `fabs(spans[1] - 966.7)` was 103.3, outside 10%. |
| E6 | The first `getFor` returned null within 10 s (`replay_node_host_test.cpp:1510`). Teardown then blocked in `PipelineImpl::stop` → `waitForReplayNodes`. The process was killed. Sequence checks and the 950 ms window were not reached. |
| E9 | The body reached the closing brace. Shutdown blocked with the replay thread in `ReplayVideo::run` → `Output::send` (`Replay.cpp:1459`). The process was killed. Catch recorded `SIGTERM` at the test line. The loop / warning CHECKs have no reported result. |
| E10, E11, E12 | Passed alone (exit 0). |
| G1 | `Replay decode` info count is 10. After `stop()`, `getHostCacheInfo().entries.size()` is 0. The catalog requires one `DECODED` entry whose `sourceHash` is `hashDir(source)`. |
| G2 | The second `start()`, after `stop()` and `wait()`, logs `ReplayVideo source is not prepared` (`Replay.cpp`) and never reaches `Replay finished`. |
| G3 | `setHostCacheLimit(100000)` does not throw. The catalog requires an error containing `4608000`. The node logged `10/10` decode lines. |
| G4 | Passed. Limit `4608000` starts and `isRunning()` is true. |
| G5 | `setHostCacheLimit(4607999)` does not throw. The node logged `10/10` decode lines. |
| G6 | Passed. After `stop()` and `clearHostCache()`, the entry list is empty. G1 already observed an empty list after `stop()` without `clear`, so this pass does not show that `clear` removed a live entry. |
| G7 | After one output frame, and after the `100/100` decode lines, `entries.size()` is 0. The catalog requires one `DECODED` entry during the run. |

Filtered run (`B22*`, `B23*`, `B25*`, `E7*`–`E12*`, `G*`): Catch summary `6 cases, 4 passed, 2 failed` before `SIGTERM` on E9. The two failures were B22 and the signal. E10–E12 and G1–G7 were not part of those six cases; they were run afterwards, one filter at a time.

## Measured VD values

None. `replay_node_perf_test` links and was not executed.

## Gates

Not run. The host suite is not all green. That includes three `--order rand` runs, a load run, `ctest --repeat until-fail:20` on group E, and the mutation spot-check of C15, E3, D3, VA12, and B52.

## Python

`P1`, `P2`, `P3`, `P4`, and `P6` passed in an earlier host pytest in this effort.

`P7` waits up to 5 s for a cache entry, then calls `pipeline.stop()` before asserting one `DECODED` entry. `stop()` blocked in `PipelineImpl::waitForReplayNodes` (`Pipeline.cpp:2218`, called from `Pipeline.cpp:2429`). The process was killed after about 3.5 minutes. The `DECODED` assertion was not executed. The expected count stays 1. G7, on the C++ host, observed 0 entries during the same kind of PRE_DECODE run.

## Harness choices

- `replay_test::setFps` is the float setter. The record device TU does not import it, so unqualified `setFps` there is the record overload. The replay device TU does import it.
- `setFast` sets loop false, stop-at-end false, and fps 1000. C15, C17, C18, C22, E3, and E4 call `setFps(0)` after `setFast` so the catalog timestamps use the recorded mean interval. C16 uses `setFps(20)`. A3b uses `setFps(0)` on `META30` / `plain_h264`. VB19 and replay perf `VD2`–`VD4` leave fps unset.
- `HostReplay` creates a blocking output queue of 16. `queueOf` replaces the test's `shared_ptr` only. E6 was not given a harness change. Its unread first queue is a hypothesis for the null read and the later join, and it was not proven.
- `makeDaiFrame` creates the parent directory. `record_test::makeMcap` is qualified at record/replay call sites because both namespaces define `makeMcap`. `RecordReplayImpl.hpp` has no include guard and is included once.
- `Pipeline` is constructed from `shared_ptr<Device>`. `Pipeline(Device&)` does not exist. `cameraInfoFor` needs `<regex>`.
- `makeDeviceSource` sets transfer false and `keepOnDevice` true on its own pipeline. A `RecordingGuard` created after that snapshot will not delete the recording. Callers delete it by the parent recording name. `DeviceReplay::begin()` while an old pipeline is still alive destroys that pipeline before the old guard is reset. `dropPipeline()` resets the guard first.
- Two pipelines cannot stay live on one device. `CacheGuard` clears the cache and restores `limitBytes`. VA18 opens extra `Device` objects and restores the saved limit.
- Progress lines are `Replay upload <hash> <done>/<total> bytes` and `Replay decode <hash> <done>/<total> frames`.
- `needleCount` uses the max of the device log and the host log when both contain the needle, so one line is not counted twice.
- Holistic `camera_info.json` for device tests comes from a one-second RAW recording on the live device (`cameraInfoFor`), not from a hand-written empty object. `makeHolisticDir` writes the caller JSON and copies `plain60_h264.mp4` plus the META60-style sidecars named in the spec.
- CMake uses `DEPTHAI_HAVE_OPENCV_SUPPORT` and `DEPTHAI_ENABLE_PROTOBUF`. The device binary is `replay_node_catalog_device_test`. Labels match the record catalog (`noreplayci`, no CMake label `replay`). `RESOURCE_LOCK device` is set.
- Python host cache is `dai.node.ReplayVideo.getReplayCacheInfo` / `setReplayCacheLimit` / `clearReplayCache`. Device cache is `device.getReplayCacheInfo` / `setReplayCacheLimit` / `clearReplayCache`. `dai.ReplayCacheKind` is the Python enum.
