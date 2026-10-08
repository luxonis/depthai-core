# Multi-device sync flight recorder

Enable the host-side recorder when running the existing FSYNC or PTP tests:

```bash
export DEPTHAI_SYNC_DEBUG=1
export DEPTHAI_SYNC_DEBUG_DIR=/absolute/path/to/sync-debug
./run_multi_device_sync_tests_entrypoint.sh fsync
```

The variables are inherited by the CTest/test process. Direct invocation of
`multi_device_fsync_test` or `multi_device_ptp_test` also works. Rebuild the SDK
and the test executables together after applying the instrumentation.

If no directory is specified, artifacts go to `sync-debug/` relative to the test
executable's working directory. Each case writes a uniquely timestamped JSON
file, and prints `Sync debug artifact: <path>` to stderr.

- **Failure:** full bounded metadata histories and counters, frozen before
  exception unwinding can stop the pipeline. Interval violations freeze at
  detection, before Catch2 prints the fatal assertion.
- **Success:** configuration and counters only, frozen before pipeline stop.
- **Disabled:** no histories or artifacts; unregistered queue lookups return
  after an atomic check. No queue capacities, blocking modes, synchronization
  thresholds, timing budgets, or assertions are changed.

The implementation is private to the host SDK and these tests. No wire metadata,
public SDK API, or Python bindings are added.

## Records

Each stream has separate 512-entry rings for:

| Stage | Meaning |
|---|---|
| `arrival` | Host message reached a selected Sync input, before callbacks/enqueue |
| `enqueue` | Input queue accepted the message |
| `eviction` | This exact input message was removed by drop-oldest |
| `sync_consumption` | Sync popped this frame for timestamp matching |
| `sync_discard` | Sync replaced it during matching or abandoned/drained it on source loss/stop |
| `group_emission` | Frame is a member of a completed group before `out.send()` |
| `group_enqueue` | Completed group was accepted by the test output queue |
| `group_eviction` | Completed group was removed by output-queue drop-oldest |
| `group_dequeue` | GroupReader received this group, before validating/analysing it |

Frames contain sequence, DEVICE/SYSTEM/host-domain timestamps in nanoseconds,
exposure in microseconds, numeric FSYNC mode, and a host steady-clock event time.
Missing SYSTEM timestamps are represented as JSON null. Pixel data is never
stored in the recorder. Temporary eviction references are released after each
push has been recorded, not retained in the history.

Streams include device ID and the original device/socket/sensor input name.
For `sync_discard`, the reason distinguishes timestamp matching from source
unavailability/stop, and the candidate spread and grouping threshold are recorded.
The next `sync_consumption` entry for that stream identifies the replacement.

`groups` has separate emission, enqueue, eviction, and dequeue histories.
Each emission receives a diagnostic `group_id`; it is not written into the
MessageGroup or inferred from its inherited sequence number. Join a group event
to the corresponding per-stream stage entries by `group_id` to reconstruct its
member map. IDs use weak references and do not retain frame/group payloads.

`contexts` records phase/sample transitions and the currently last dequeued
group ID. At measurement entry, sample 0 points to the final convergence group.
After a failing gap check, the last two dequeued groups and the last context give
the previous/current member maps, observed interval, and allowed interval.

The header also includes the host UTC/system-clock and steady-clock start
anchors, freeze time, test configuration, and failure reason. DEVICE timestamps
are only comparable within the same device; use host event times to correlate
delivery and queue residence across streams.

## Counters and snapshot boundaries

Every selected queue reports arrivals, accepted enqueues, exact eviction count,
rejections, zero-capacity discards, capacity/blocking mode, and high-watermark.
Each history reports its lifetime `total` and `history_overwrites` independently.
At 60 FPS, a 512-entry frame/group stage ring covers approximately 8.5 seconds;
busy matching/discard streams can have shorter coverage. Phase/sample context
has its own bounded ring.

Queue mutation times are captured while the queue lock is held. Metadata is
recorded after unlocking; consumers may therefore record a dequeue before its
producer records enqueue completion. Match identities and recorded times, not
JSON append order. `pushes_without_completion_record` identifies arrivals whose
completion was still in flight or aborted when the recorder froze.

All recording updates are synchronized. Freeze excludes subsequent updates,
but it is not a transactional stop of all SDK threads. Check incomplete pushes,
`recording_errors`, and `unmatched_group_ids` before asserting exact accounting.
An unmatched ID is zero; retained member identities/timestamps can still be used.
Diagnostic-copy failures are counted without changing queue overwrite behavior.
Artifact write errors are printed without replacing the original test failure.

## Interpreting a gap

1. Missing frames already at `arrival`: investigate device output/transport with
   the corresponding device-side logs.
2. Arrivals present with matching `eviction` records: input-queue overwrite.
3. Input frames consumed but discarded while matching: Sync selection; inspect
   which stream/timestamp forces the replacements.
4. Groups emitted continuously with exact output evictions: output-queue overwrite.
5. Emissions and enqueues present but delayed consumption: host scheduling/reader
   delay; use queue counters and host steady-clock times.
6. DEVICE timestamps/sequence continuous but SYSTEM timestamps jump: investigate
   timestamp publication/translation rather than assuming physical capture loss.

This recorder adds bounded CPU/mutex overhead when enabled. It does not establish
that every sequence number corresponds to a physical sensor trigger. Compare
with device-side capture/output evidence and passing counter summaries.
