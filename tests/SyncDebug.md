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
- **Success:** configuration, counters and delivery-duration maxima only, frozen
  before pipeline stop.
- **Disabled:** no histories or artifacts; unregistered queue lookups return
  after an atomic check. No queue capacities, blocking modes, synchronization
  thresholds, timing budgets, or assertions are changed.

The implementation is private to the host SDK and these tests. No wire metadata,
public SDK API, or Python bindings are added.

## Records

Each stream has separate 512-entry rings for:

| Stage | Meaning |
|---|---|
| `arrival` | Selected Sync input's arrival recorder timestamp, inside the State lock, before callbacks/enqueue; raw send entry is recorded separately |
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
Each context also has an artifact-local `context_id` for joining reader operations.

The header also includes the host UTC/system-clock and steady-clock start
anchors, freeze time, test configuration, and failure reason. DEVICE timestamps
are only comparable within the same device; use host event times to correlate
delivery and queue residence across streams.

## Output delivery timings (schema 1 extension)

The optional `output_timings` object appends diagnostics to schema version 1;
existing stage histories/counters remain available. Each selected output/session
has one preallocated 512-entry ring shared by Sync and MessageQueue timing data.
Join its `events` to group headers and per-stream member maps by `group_id`.
`record_id` identifies the timing record itself, including unmatched groups
(`group_id: 0`) and repeated sends. No image/group ownership is stored there.
Completed summaries use their reserved record IDs, so a consumer freeing its
group before the producer returns cannot invalidate the association.
Repeated sends of the same group share its group ID; consumer timing attaches to
the latest retained delivery record, whereas producer summaries use record IDs.
`reader_record_id` links that delivery to the separate `reader_timings` record.
Consumer updates cannot overwrite a producer's reserved record ID or send summary.

All checkpoint fields ending in `_ns` are **host steady-clock timestamps in
nanoseconds**, in the same domain as the stage histories. `durations` and
`duration_maxima.*.max_ns` are **elapsed nanoseconds**, not timestamps. Null means
a checkpoint/duration is unavailable or unpublished. Negative/unavailable
intervals are excluded from maxima, rather than converted to zero.

### Checkpoint boundaries

| Object / fields | Exact boundary |
|---|---|
| `sync.before_emission_recorder_ns`, `after_emission_recorder_ns` | At the Sync call site immediately before/after `emitted()`, including recorder mutex wait and member-metadata collection |
| `sync.before_output_send_ns`, `after_output_send_ns` | Immediately before actual `out.send()` and immediately after normal return, inside the surrounding `outputBlockEvent` scope |
| `sync.output_send_exception_ns` | In Sync's catch handler if `out.send()` throws; the normal-return checkpoint remains null |
| `message_queue.entry_ns` | At send entry before diagnostic registry lookup, validation, queue-closed checks or arrival recording |
| `message_queue.after_registry_lookup_ns` | Immediately after the selected queue's registry lookup returns, before start-record publication |
| `message_queue.before_arrival_recorder_ns`, `after_arrival_recorder_ns` | At the send call site around `arrival()`, including its State lock wait and metadata recording |
| `message_queue.before_callbacks_ns`, `after_callbacks_ns` | Immediately around `callCallbacks()`, including its callback-mutex acquisition and all callbacks |
| `message_queue.before_push_ns`, `after_push_ns` | Immediately around the underlying push call; return is captured before post-push diagnostic recording |
| `message_queue.lock_wait_started_ns`, `lock_acquired_ns` | Immediately before acquiring the push queue guard and immediately after acquisition |
| `message_queue.capacity_wait_started_ns`, `capacity_wait_finished_ns` | Around the blocking condition-variable wait, after any existing BLOCKED callback and before any CANCELLED callback; includes predicate checks and final guard reacquisition, also on timeout/destruction |
| `message_queue.enqueue_started_ns`, `enqueue_completed_ns` | Immediately before `queue.push()` and after mutation/size capture, while guarded. At zero capacity these instead bracket clearing/discarding, not delivery |
| `message_queue.guard_released_ns` | First scalar timing-scope checkpoint after guard destruction, also on early return/exception; on success precedes `signalPush.notify_all()` |
| `message_queue.notify_started_ns`, `notify_finished_ns` | Immediately before/after the actual `signalPush.notify_all()` call, outside the queue guard, in all four push variants (copy/move, regular/timed) |
| `message_queue.after_push_recording_ns` | Immediately after queue counters, eviction identities and enqueue/member metadata have been recorded after unlocking |
| `message_queue.before_listeners_ns`, `after_listeners_ns` | Immediately around `notifyListeners()`, including its existing notifier mutex and notifier calls; absent when a timed send is rejected |
| `message_queue.exit_ns` | At stack-operation destruction on normal return or exception, after listener notifications where applicable, before publishing the final send summary |
| `dequeue_recorder.begin_ns`, `metadata_finished_ns` | Entry to `dequeued()` before State mutex acquisition, through group/member recording and timing-record lookup, while still inside that lock |

Both regular and timed `MessageQueue::send()` paths are covered. `trySend()` uses
the timed path; its own preflight closed-queue check is outside these boundaries.
`message_queue.timed` identifies the timed path. `accepted` is the underlying push
result, or null if it never returned; it does not override zero-capacity discard
semantics. `exception` is true for unwinding (closed/null message, callback,
push/event callback or listener failure); it is null until the summary is published.
No exception is swallowed or substituted by this instrumentation.

`start_published_ns` / `summary_published_ns` are separate publication checkpoints
taken inside the recorder mutex. They are not substitutes for the call-site
checkpoints. The publication timestamp precedes the remaining scalar summary
accounting and lock release. `exit_ns` excludes this final publication; Sync's
whole `output_send_ns` includes it, fan-out sends and output event bookkeeping.
The Sync `recorder_to_output_send_ns` interval includes surrounding output-block
event creation and, with multiple selected outputs, other emission recorders.

### Durations and summaries

Per-record `durations` and lifetime `duration_maxima` report the recorder,
recorder-to-send, whole output-send, entry-to-callback, callbacks,
callbacks-to-push, initial queue-lock wait, capacity wait, guarded interval,
enqueue action, unlock-to-push-return, producer post-push recording, send tail,
send-summary publication and consumer dequeue-recording intervals. Each maximum
has a `samples` count; it includes completed operation summaries even after their
history entries have rolled out. Successful artifacts omit `events` and keep these
maxima/counters.

`queue_guard_interval_ns` is acquired-to-release elapsed time.
`queue_guard_excluding_capacity_wait_ns` subtracts the entire condition-variable
wait interval (including its predicate/reacquisition work), when present. This
separates capacity waiting from the initial lock wait and the rest of the guarded
execution; it is an elapsed-time estimate, not exact CPU or mutex-ownership time.
For nonblocking outputs there is no capacity wait. Pre-enqueue eviction and
existing queue event callbacks are inside the guarded interval; the SUCCESS
callback is after `enqueue_completed_ns`. Unlock-to-push-return includes notify.
That interval now splits into `unlock_to_notify_ns`, `push_notify_ns`, and
`notify_to_push_return_ns`. The middle interval brackets the call, but still
includes any preemption between its two clock reads. Zero-capacity early return,
timeout, closure, or an exception before notification leave notify checkpoints
null; existing notification behavior is preserved.
`producer_push_recording_ns` measures post-push recorder work including State
mutex wait. `send_tail_ns` covers listener notifications and result/exception
handling. Consumer `dequeue_recording_ns` includes recorder-lock wait/metadata,
but excludes the earlier queue pop/diagnostic lookup and the final lock release.

### Pending operations and freeze

A start record is published before emission member metadata or send validation /
callbacks. End summaries are published once the operation finishes, under the
State mutex. `sync_started` / `send_started`, `sync_complete` / `send_complete`
and their `*_inflight` flags distinguish known starts from published summaries.
Here **inflight means summary missing**, not proof that the SDK operation was
still executing at freeze. A completed send can still await its recorder lock.

To keep checkpoint overhead small, an unfinished record contains only its start
timestamp(s), plus any independently completed send/dequeue data; intermediate
checkpoints remain on the producer stack until its end summary. Null later fields
do not prove those phases never ran. If a consumer freezes before a producer
summary, subsequent summaries are excluded, without delaying freeze or delivery.
For example a failing group's `group_enqueue`/`group_dequeue` may exist while its
Sync or send timing record is incomplete. Earlier completed groups, such as the
previous group in an interval failure, retain their full timing breakdown.

Starts are also announced with scalar atomics before waiting for the State mutex.
`sync_starts` / `send_starts`, `*_start_records`, `*_summaries_published`,
`*_starts_without_record`, `*_without_summary`, and `last_*_start_ns` expose starts
whose initial publication lost the race to freeze. These announcement counters
are sampled once at freeze; boundary races are nontransactional, and a last start
newer than the freeze timestamp is reported as null. A send still waiting for its
registry lookup has not announced a send start yet; the encompassing Sync start
remains the useful pending context. No queue-guard lock acquires a recorder lock.

`total` / `history_overwrites` count timing-ring records, not checkpoints or
groups. `overwritten_pending_records` counts evicted records with a missing
summary, and `completion_updates_without_retained_record` counts summaries whose
reserved slot rolled out or whose start could not be published. Check these,
`recording_errors`, `unmatched_group_ids` and existing
`pushes_without_completion_record` before making exact-accounting claims.

When no session is registered, sends take one atomic fast-path check and make no
new clock calls. While a session is registered, send entry is captured before
lookup (also for unselected queues); only selected inputs and outputs retain timing
records and collect the subsequent checkpoints. Queue push timing clocks run only
when the private diagnostics timing flag is enabled. Recording performs no JSON
construction or file I/O; both occur only when finishing the frozen session.

## Reader timings (schema 1 extension)

`reader_timings` has a separate preallocated 512-entry, metadata-only ring for
**every GroupReader read operation**: startup, warmup, convergence, measurement,
expired-deadline reads, queue timeouts, closure and assertion/SDK exceptions.
The reader caches its selected output handle at construction, so its earliest
entry does not need a registry lookup. A private friend bridge uses the same
MessageQueue `get<T>()` implementation with optional stack-only timing pointers.
Public signatures, object layouts, wire types and bindings are unchanged.
Ordinary SDK gets pass null pointers and introduce no timing clocks.

`record_id` is local to `reader_timings`; `delivery_record_id` refers to
`output_timings.events[].record_id`. `group_id` joins the emitted group and the
existing per-stream `group_dequeue` member maps. The popped **uncast** handle is
captured before casting/event bookkeeping and retained only until this read ends.
This preserves exact identity even if a typed cast is null or later bookkeeping
throws. `popped_frame` provides inherited Buffer metadata (or `valid_buffer:
false`) when group identity cannot be matched; it is not a MessageGroup member
map. Missing IDs are zero. Repeated sends use the latest retained delivery record,
while each read keeps its own stable record ID.

| Fields in each reader event | Exact boundary |
|---|---|
| `context_id`, `phase`, `sample_index` | Last context supplied by this reader, copied at read entry; includes measurement sample 0 and each next-sample context |
| `context_entry_ns`, `context_returned_ns` | Around context recording, including State lock acquisition and release; the return checkpoint is immediately before `setDebugContext()` returns |
| `entry_ns` | First opt-in reader checkpoint, before start-record publication and the existing steady deadline check |
| `start_published_ns` | Reader start record reserved under State lock, before the deadline check; the publication overhead is explicit |
| `deadline_ns`, `deadline_checked_ns` | Caller-supplied absolute steady deadline and the original `now` used for the comparison/`deadline - now` timeout |
| `get.entry_ns` | At entry to the shared timed-get implementation, before its preflight closed-queue guard check and pipeline event creation |
| `get.before_pop_ns` | Immediately before the primitive timed-pop call |
| `get.lock_wait_started_ns`, `lock_acquired_ns` | Immediately before/after actual pop-guard acquisition |
| `get.wait_started_ns`, `wait_finished_ns` | Around the original predicate-based condition-variable wait; return includes predicate evaluation and final guard reacquisition, also on timeout/closure |
| `get.pop_completed_ns` | Immediately after moving the actual front handle and removing it, while guarded, before diagnostic metadata publication |
| `get.guard_released_ns` | First scalar checkpoint after guard destruction, including timeout/closure/unwinding if the guard was acquired |
| `get.after_pop_ns` | Primitive return at the MessageQueue call site; includes the existing post-unlock `signalPop.notify_all()` on successful pop |
| `get.cast_finished_ns` | Immediately after the dynamic cast, after saving the exact uncast popped handle |
| `get.return_ready_ns` | Just before timed-get return, after event size/cancel bookkeeping; event-scope destruction can still follow |
| `typed_get_returned_ns` | GroupReader call site after typed get actually returns, before the timeout check and any registry lookup |
| `before_registry_lookup_ns`, `after_registry_lookup_ns` | Immediately around the original post-get lookup; registry lock is released before any State update |
| `before_dequeue_recorder_ns`, `dequeue_metadata_finished_ns`, `after_dequeue_recorder_ns` | Before recorder entry, after group/member metadata and delivery-record matching under State lock, and after recorder return/lock release |
| `before_analysis_ns`, `analysis_finished_ns` | After the non-null group assertion through all member validation, SYSTEM timestamp collection, spread/median analysis, and normal analysis return |
| `exit_ns`, `summary_published_ns` | Logical stack-operation exit, then end-summary publication under State lock, including assertion/SDK exception unwinding |

`deadline_expired: true` means no queue get was attempted. `get.pop_result` is
`popped`, `timeout`, `closed`, or null when unavailable. `timed_out` is the timed-get
out parameter after normal return; closure still throws with the original
out-parameter semantics. `registry_lookup_found` identifies a completed lookup
that no longer found the selected queue; dequeue-recorder checkpoints stay null
in that case. `get.null_cast` is populated only if casting finished,
and can be true after a successful primitive pop. `exception` is populated only
in a completed summary. Exception paths retain the checkpoints actually reached;
later checkpoints remain null. No timeout is extended, wait is replaced, or
deadline is rechecked after a successful pop. A preempted pop can still return
after the caller's deadline, as before.

`duration_maxima` and failure-event `durations` include context-call,
context-return-to-entry, start publication, deadline-check/get setup, initial
pop-lock wait, predicate wait/wake, wake-to-pop, pop-to-unlock, primitive return,
cast/typed-return, registry lookup, dequeue recording, analysis, logical read tail,
entry-to-exit and final summary-publication intervals.
The shared timed-get setup interval includes its existing `isDestroyed()` guard
check and event creation; the separately measured lock wait is the actual pop
guard. `pop_wait_ns` is not proof the queue was empty throughout the interval.
Within a phase without a fresh context on each read, context-to-entry is elapsed
time since that phase's context, rather than an immediate next-sample delay.
All maxima are independent samples and must not be added as one operation.

Read starts are announced atomically **before** start publication/State lock wait.
`starts`, `start_records`, `summaries_published`, `starts_without_record`,
`without_summary`, `last_start_ns`, `overwritten_pending_records` and
`completion_updates_without_retained_record` follow the output pending-summary
conventions. Events have `started`, `complete`, `inflight` flags. Intermediate
read checkpoints stay on the stack until the end summary; a pending start is not
an assertion that a later null checkpoint was never reached. Freeze may exclude
an end summary even if the pop or analysis already completed. Read maxima include
completed summaries whose reserved records have rolled out. Successful artifacts
keep counters/maxima and omit the event ring.

## Input delivery timings (schema 1 extension)

Each selected stream now has `input_delivery_timings`: one preallocated
512-entry ring using the same scalar SendTiming/final-summary pattern as output
delivery. Only the Sync inputs explicitly registered by this test session are
retained. Twelve asynchronous producers announce starts with per-stream atomics
and publish records under the existing State mutex; no extra callbacks, recorder
locks under queue guards, per-frame JSON, or per-frame file I/O are introduced.

Events carry stream-local `record_id`, `phase`, `sample_index`, `frame`,
`message_queue`, `durations`, and `started` / `complete` / `inflight`. Join `frame`
to stage histories by stream identity, sequence, DEVICE and SYSTEM timestamps.
Metadata is copied in the start recorder before arrival recording; no payload
handle is stored in the ring. `message_queue` uses the same checkpoint fields as
output sends, including the new raw `entry_ns`, `after_registry_lookup_ns`,
`before_arrival_recorder_ns`, and `after_arrival_recorder_ns` boundaries. Raw entry
is a MessageQueue send entry immediately downstream of the relay, not raw XLink
read/parse completion. It includes neither device clocks nor a transport probe.

The legacy `arrival.events[].steady_ns` remains the timestamp taken **inside the
arrival recorder's State lock**. It has not been changed to send entry time.
Replay raw send entries separately: a gap already between entries precedes this
arrival recorder; a prompt entry followed by late lookup/arrival points into the
measured host region. `send_registry_lookup_ns` measures entry-to-lookup-return;
`lookup_to_arrival_recorder_ns` includes start-record reservation, metadata,
validation and any existing pre-arrival closed check; `arrival_recorder_ns`
includes arrival State-lock wait and recording. The latter two can be substantial
even when transport delivered a frame promptly. Entry gaps alone do not select
transport versus earlier relay scheduling, and DEVICE holes still require
same-device capture/request evidence.

Input summaries have the same generic start/summary/overwrite counters as reader
summaries and independent send-duration maxima, including callbacks, queue push,
notifications and final summary publication. Raw entry/lookup are present in a
reserved pending start even if freeze excludes arrival or send completion.
A send still waiting on the **registry** has not selected its stream or announced
its per-stream start yet; its entry checkpoint remains unpublished. The absence
of a retained entry therefore cannot prove absence of transport delivery.

## Extension and artifact identity

The format remains `schema_version: 1`. New artifacts contain
`diagnostic_extensions` with `output_notify_boundaries_v1`,
`reader_boundaries_v1`, and `input_delivery_boundaries_v1`. Check those markers
and the new objects when identifying instrumented captures; an SDK version string
alone does not identify these changes. Existing filename/start anchors and the
printed artifact path identify a case. Reader IDs and input IDs are local to
their respective rings in that artifact, not wire sequence numbers or device
request IDs. Preserve the corresponding source/executable identity with each
user-run capture.

All extensions use host steady time for cross-stream event correlation. DEVICE
timestamps are compared only within their device. Snapshot/freeze remains
nontransactional across threads: scalar start counters are copied once, newer
last-start timestamps are null, and checkpoints unpublished by that boundary
are excluded. Elapsed time includes scheduling/preemption and does not establish
CPU cost or a mutex owner. At 60 FPS each 512-record input/read ring typically
covers about 8.5 seconds; repeated sends/timeout reads can shorten its coverage.

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

1. Missing frames already at `arrival`: first compare raw selected input delivery
   entry/lookup/arrival checkpoints to separate host recorder delay from an
   earlier delivery gap, then correlate the corresponding device-side logs.
2. Arrivals present with matching `eviction` records: input-queue overwrite.
3. Input frames consumed but discarded while matching: Sync selection; inspect
   which stream/timestamp forces the replacements.
4. Groups emitted continuously with exact output evictions: output-queue overwrite.
5. Emissions and enqueues present but delayed consumption: host scheduling/reader
   delay; use queue counters and host steady-clock times.
6. DEVICE timestamps/sequence continuous but SYSTEM timestamps jump: investigate
   timestamp publication/translation rather than assuming physical capture loss.
7. Long emission-to-enqueue interval: use the same group's `output_timings` to
   separate emission-recorder time, setup before actual output-send entry,
   entry/arrival recording, callbacks, initial queue-lock wait, capacity wait,
   enqueue and post-push recording. The legacy emission group header is recorded
   inside the emission recorder, before its member metadata; that interval alone
   does not locate the stall inside `out.send()` or establish a mutex blockage.
8. Long sample-context-to-dequeue interval: join `reader_timings` by context and
   group IDs to locate context return, reader entry, deadline/get setup, actual
   guard/wait/wake/pop, typed return, registry lookup and dequeue recording.
9. Long post-unlock producer interval: compare notify start/end and push return
   before attributing the entire interval to `signalPush.notify_all()`.

These elapsed intervals locate an execution boundary, not a CPU/scheduling cause
or a mutex owner. Callback duration includes its mutex wait and possible thread
preemption. Compare the producer and consumer recorder overhead with the input
arrival/capture timelines before assigning a cause.

This recorder adds bounded CPU/mutex overhead when enabled. It does not establish
that every sequence number corresponds to a physical sensor trigger. Compare
with device-side capture/output evidence and passing counter summaries.
