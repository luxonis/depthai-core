# TDD evidence report: multi-device support (feat/multi_device_support)

Source plan: `docs/design/multi_device.md` (committed as the first commit of this work).
Journeys were taken from the plan's goals section; validation commands below were run
against two/three real RVC4 devices (10.12.234.143 = 733979940, 10.12.234.183 =
142384345, 10.12.234.100 = 2437739507) with `PYTHONPATH=build/bindings/python` and the
`depthai3-12` mamba env. Test scripts live in the session scratchpad; the durable
regression guard is `tests/src/onhost_tests/pipeline/multi_device_pipeline_test.cpp`
(ctest label `onhost`, deliberately without `ci`).

## User journeys (from the plan)

1. Fan-in: N devices stream into host nodes of one pipeline with one start/stop.
2. Device-to-device links relay through the host automatically.
3. Partial operation: losing one device idles its streams, the rest keeps running,
   the lost device reconnects on its own; state is observable.
4. Cross-device software sync via a host Sync node on host-comparable clocks.
5. Per-device configuration overloads; source-device introspection for host nodes.
6. Existing single-device programs behave identically (regression suite green).

## RED/GREEN log (checkpoint commits on this branch)

| Cycle | RED evidence | GREEN evidence | Commit(s) |
|---|---|---|---|
| Python bindings compile | `cmake --build build --target depthai` failed: `addBetaNode` pushes a 2-arg lambda into the 3-arg `pyNodeCreateMap` (hard compile error, 21 beta TUs) | same target builds, module imports, beta module present | 850044cfa |
| Registry + link error | compile-time RED: `Pipeline::addDevice/getDevices` did not exist; runtime RED: cross-pipeline `link()` accepted (test FAILED 1/4) | `multi_device_pipeline_test` all green; hardware registry test PASS (master promotion, dedup, fan-in 143/141 frames) | b5df715a9 (RED), c30d2821b (GREEN) |
| Per-device build/start/stop | baseline wheel: `RPC 'startPipeline' failed: Camera on socket 0 already exists` for a 2-device pipeline | 2-device fan-in streams; record/replay rejected; `getSourceDevice` resolves; parallel start 0.15 s | c30d2821b |
| Relay | runtime RED: `Direct connection between device nodes ... not supported in this MVP` | Camera(A)->ImageManip(B)->host: 55 processed 320x200 frames; A's direct output unaffected | 522b09754 |
| Partial operation | old behavior: any device loss tears down the whole pipeline (monitor calls global disconnectXLinkHosts; bridge nodes throw) | close(B): A streams 136 frames, B idle, pipeline running, FAILED callback; crash(B): DISCONNECTED->RECONNECTING->RUNNING in 16 s, frames resume; last-device close stops pipeline | 2cb6c29a5 |
| Sync hardening | old Sync blocks forever on a dead input; DEVICE source across devices accepted | DEVICE-source rejected at build; crash(B) mid-sync: 0 groups while degraded, no deadlock, groups resume at ~29/s after reconnect | 46dd6a8f1 |
| Per-device setters | overloads did not exist (AttributeError / no overload) | hardware: per-device calibration (distinct boards OAK4-D-PRO-W vs OAK-4-S-W), board-config isolation, non-member device rejected | 134ed9c0b |
| DeviceInfo hardening | unit RED implicit (APIs/fields absent) | 8 host-only unit cases pass; hardware connect-by-IP with TCP_IP pinned | d10c9ad71 |
| Examples | n/a (examples are the plan's own verification vehicle) | all 6 ran on hardware: relay 1123/838 frames, host-node mosaic 1588/1263 frames, frame-sync 1130/980 groups (~14 ms spread) | 9853fd6bc |
| Three devices | n/a | 3 devices + relay in one pipeline: 74-76 frames per stream, start 0.31 s | (final validation) |

## What the passing tests guarantee

- `multi_device_pipeline_test` (host-only, no device): cross-pipeline links throw,
  same-pipeline links work, host-only pipelines report no devices, null addDevice is
  rejected, DeviceInfo IP/id/local parsing semantics.
- onhost ctest suite (39 tests): repeatedly 100% green after every stage - the
  single-device regression guard the plan demands.
- On-device `sync_test` binary against a real RVC4: rewritten Sync preserves
  single-device behavior.
- Hardware scripts (scratchpad `test_*.py`): every plan behavior listed in the
  journeys, on real devices, including crash-injected reconnects
  (`DEPTHAI_CRASH_DEVICE=1` + `crashDevice()`).

## Known gaps / intentional deviations

- XLink-side fixes (design doc PR 1) belong to luxonis/XLink - not in this repo.
- fd-forwarding boundary rule is implemented but not hardware-exercised here (needs a
  local shdmem transport; this rig is TCP-only).
- Per-device pipeline-debugging aggregators not implemented (multi-device debugging
  keeps its MVP rejection).
- Sync threshold semantics: free-running 30 fps cameras have an arbitrary inter-device
  phase; a <10 ms threshold syncs only by phase luck. Tests/examples use 17 ms
  (half the frame period); hardware FSYNC/PTP is the path to tighter sync.
- Full coverage tooling (gcov) was not run; verification is by the suites above.
