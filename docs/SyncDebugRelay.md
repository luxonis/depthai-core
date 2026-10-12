# Selected Sync host-relay flight recorder

The private `relay_boundaries_v1` extension adds XLink receive/parse/relay-send
observations to the existing **schema 1** Sync recorder. Enable
`DEPTHAI_SYNC_DEBUG=1` in the host process before the existing diagnostic Session
is created; `DEPTHAI_SYNC_DEBUG_DIR` selects its artifact directory. The Session
must register its selected Sync input queues before `Pipeline.start()`. Firmware
recording independently requires `DEPTHAI_CAMERA_DEBUG=1` in the firmware launch
environment; a host export does not establish firmware enablement.

## Selection and private API

At `XLinkInHost::run()` setup, `out.getConnections()` supplies canonical Node::Input
queue addresses and `out.getQueueConnections()` supplies explicit output-queue
addresses. The private `syncdebug::relayContext` resolves only queues already in
the Session registry and excludes the selected Sync output. It caches all selected
fanout recipients once, deduplicating queue handles. This observes the bridge
connections after Pipeline's existing device-to-host rewrite; no per-frame global
queue scan, registry lookup or registration callback is introduced. A direct
relay stream connected to two registered Sync inputs contributes the same native
observations to both inputs' independent histories.

Each selected stream's `relay_timing.sources` maps source index to actual
`stream_name`, `relay_node_id`, and registered `device_id`. Dynamically generated
bridge names, including `__x_<source node>_dynamicOutputs_<output>`, identify the
channel. Names and source mappings are allocated at worker setup only. Contexts
cache safe State handles; Session destruction freezes them before unregistering
queues. Frozen handles are inert even if a relay is finishing or reconnecting.
Selection is fixed for that worker run: a Session created after it starts cannot
retroactively observe earlier reads. The existing test registration order needs
no new GroupReader or test-helper hook.

Each mapping also preallocates one scalar atomic native-progress slot. Cached
contexts own that slot directly, so another relay adding a source mapping cannot
invalidate a running worker's slot. Native packet/parser/send entry and return
clocks publish to it without State::mutex or registry lookup. Freeze takes one
bounded revision-checked sample; it never spins waiting for the producer. The
tuple uses C++17 sequentially consistent atomics. On a platform where those
atomics are not lock-free, this extra publication is disabled and reported as
unavailable; ordinary ring recording remains portable.
Packet/parser `exit` checkpoints include exception unwinding; they do not alone
establish successful parse or receive. Exception counters and the operation's
later ring summary provide that additional coverage when retained.
Entry checkpoints are caller-side pre-call boundaries. A worker can be preempted
after publication and before invoking the API; a retained entry identifies that
region, not proof that the API itself is blocking.

`src/pipeline/SyncDebug.hpp/.cpp` contain the additive, private API:

- `RelayContext` and `relayContext(queues, stream, node)` cache selected handles;
- `RelayOperation` records a stack operation without retaining a message;
- `RelayPartOperation` uses a thread-local borrowed operation to instrument native
  read/parser calls, including recursive group and packetized reads.

No installed/public class layout, binding, wire metadata or message payload changes.

## Field map

Every selected input gets a separately preallocated **512-record** `relay_timing`
history. It uses the existing phase/sample key and operation-history start,
summary, overwrite and incomplete-publication counters. Success artifacts retain
source mappings, counters and independent duration maxima; failure artifacts
also contain the retained scalar events. `record_id` is local to that input's
relay history, not a serialized message ID or a Sync group ID.

| Field | Boundary |
|---|---|
| `operation` | `read_and_send`, `stream_open`, `reconnect_wait_after_open_failure`, or `reconnect_wait_after_read_failure`. |
| `entry_ns` / `start_published_ns` | Worker operation entry is captured **before State::mutex**; start publication follows the selected-recipient updates. |
| `last_packet_begin_ns` / `last_packet_end_ns` | Last native `stream->readMove()` entry/exit. A read interval includes waiting for a payload, SDK work and possible scheduling; it is not proof of slow parsing or network transfer. |
| `last_parse_begin_ns` / `last_parse_end_ns` | Last StreamMessageParser call, separated from packet waits. Group recursion and packetized payload gathering can invoke several native reads/parses. |
| `last_packet_recorder_begin_ns` / `last_parse_recorder_begin_ns` | Intent publication before the actual API-entry clock, separating State-lock/publication overhead from the native read/parser interval. |
| `packet_read_exits`, `parse_exits`, exception counters | Counts of exited native spans, including exception unwinds, not counts of successfully delivered frames. |
| `read_returned_ns` | Complete `readData()` returned after recursive assembly. |
| `metadata_available_ns` / `metadata_extraction_finished_ns` | Available parsed message followed by scalar extraction; extraction occurs outside the recorder's State lock. |
| `before_send_recorder_ns` / `before_out_send_ns` | Intent publication precedes the actual out.send entry clock, allowing its recording interval to be distinguished from SDK send. A freeze during intent publication can retain intent without actual entry. |
| `after_out_send_ns` | Existing `out.send(msg)` returned. This includes selected MessageQueue registry/arrival work, callbacks and delivery to all actual recipients. It establishes no queue-consumption or driver ACK. |
| `exit_ns` / `summary_published_ns` | Operation's logical exit followed by final recorder publication. Exceptions propagate through the original handling. |
| `frame` | Existing scalar sequence, DEVICE, translated host timestamp, optional SYSTEM timestamp, exposure and FSYNC metadata; no image reference. |
| Source `native_record_id`, `native_boundary`, `native_boundary_ns` | Last atomically sampled native operation checkpoint at freeze, keyed to that input's relay record ID; it survives an API call blocked before ring-summary publication. |
| Source `native_boundary_available`, `native_boundary_sample_coherent` | Lock-free support and a successful single-sample tuple before the cutoff. A racing/late sample is null, rather than interpreted as a pending API. |

Duration maxima cover aggregate readData, summed native packet-read API spans,
summed parser spans, metadata extraction, before-send recording, actual out.send,
total operation and summary publication, plus summed packet/parser start-recording
intervals. Start and post-span recording are outside the summed native read/parser
intervals but inside the overall readData/relay duration. Native intervals include
their bounded scalar atomic publication cost. These elapsed intervals can include
preemption and instrumentation. Reconnect/open operations have no frame metadata.
`exceptions` counts finalized exceptional operations and `out_send_returns` counts
observed relay-send returns. Neither substitutes for a frozen/overwritten summary.

## Firmware / host correlation

Use the actual device identity + stream + sequence + DEVICE timestamp to join the
firmware's camera-source/resize/transport checkpoints to host `relay_timing.frame`.
Check SYSTEM presence/value and source mappings as additional consistency fields.
Sequences can reset between streams or firmware processes. Firmware buffer keys
also require PID/process-session/camera-session and the HAL reservation frame.
Host record and phase/sample keys locate the corresponding selected Sync input
delivery, consumption and group/read histories.

Separate these native observations:

1. Accepted firmware resize handoff → actual worker dequeue, conversion/apply,
   source cleanup and HAL destructor/pool return.
2. Copied-output transport serialization → individual native write-API returns.
3. Host relay packet-read entry/return → parser/assembly → actual relay out.send.
4. Selected MessageQueue raw entry → arrival/enqueue → Sync → GroupReader.

Cross-process startup clock anchors do not alone establish one-way transport
latency. Long packet-read intervals can reflect no available payload, device-side
delay, receive work or scheduling. Driver queues/ACKs inside transport dependencies
are unobserved. No source timing or retaining owner is inferred from a missing join.

## Coverage and verification

Missing checkpoints are exported as null, and incomplete records mean completion
was not published before freeze. Packet/parser partial state is published at their
boundaries. Intent is published before the actual API-entry clock; during an
in-progress call a freeze can retain recorder intent with the actual entry still
unpublished in its ring. The source's coherent native-progress sample independently
locates the latest API entry/return or logical exit, using the same record ID.
That distinction is explicit rather than assigning State-lock time to the native
API. Freeze may precede the next ring publication even when the SDK call
has already returned. Histories
overwrite independently and counters expose lost start/summary associations.
The setup mapping is bounded to 64 selected recipients per relay and 64 source
mappings per input; overflow increments `recording_errors`. Non-Buffer parsed
messages have `valid_buffer=false`; no invented frame join is supplied. A setup
allocation failure increments `recording_errors` once in each active selected
Session because source selection may not yet be resolvable, then leaves relay
execution on its original path.

The disabled or unselected hot path adds no diagnostic clocks, allocations or
JSON work. Selected relay operations copy scalar metadata into preallocated rings
and use the existing recorder mutex; they never hold that mutex around SDK calls,
reconnect waits, queue sends or unknown callbacks. Source exceptions, queue close,
connection parking and callback policy remain those of XLinkInHost.

Verification was source/manual review, pinned clang-format **18.1.8** on changed
ranges, `git diff HEAD --check`, and the host datatype/protobuf consistency gates.
No compilation, configuration, clang-tidy, synthetic test, capture, hardware run
or deployment was performed. Later authorized integration/captures must establish
runtime timing overhead, scheduling and retained-window coverage.
