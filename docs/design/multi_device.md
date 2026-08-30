# Multi-device support in a single `dai::Pipeline`

|        |                                                              |
| ------ | ------------------------------------------------------------ |
| Status | v1 implemented on `feat/multi_device_support` (see appendix) |
| Target | depthai-core 3.x (next minor after 3.9.0), additive API      |
| Author | Aljaž Konec                                                  |

This document records the design decisions for running **several devices inside one
`dai::Pipeline`**, the backend changes they require, and the ordered plan of PRs that
implements them. It was produced from a structured design interview; every decision below
is deliberate, and the few places where a choice was *assumed* rather than decided are marked
**(assumption)**.

---

## 1. Goals and non-goals

### Goals (v1)

1. **Fan-in.** N devices each run their own device-side subgraph; all their outputs land in
   host nodes of the *same* pipeline object with a single `build()/start()/wait()/stop()`.
2. **Device→device links.** An output on device A may be linked to an input on device B; the
   framework inserts a host-side relay automatically.
3. **Partial operation.** Losing one device does not stop the others; the lost device`
   reconnects on its own and the rest of the pipeline keeps streaming.
4. **Clock-domain and software sync.** Timestamps from different devices are comparable on
   the host clock and the host `Sync` node aligns them safely.
5. **RVC4 standalone.** The pipeline process may itself run on an RVC4 (local shared-memory
   transport) with further RVC4s attached over the network. The local device may play any
   role, not necessarily master.
6. **Source compatibility.** Every existing single-device C++/Python program compiles and
   behaves identically. The existing test suite is the regression guard.
7. **Platforms.** RVC2 and RVC4. Homogeneous pipelines (all-RVC2 / all-RVC4) are the
   supported tier; mixed pipelines are best-effort.
8. **Scale.** Data structures are N-ary with no artificial cap; tuning and examples target
   2–4 devices.

### Non-goals (v1)
  
- Hardware sync (FSYNC / PTP) — existing configuration is documented, nothing new.
- Cross-device sequence numbers / frame ids.
- Hot-plug of a *new* device into a running pipeline (reconnect of a *known* device is in).
- Pipelines spanning several host machines.
- Holistic record/replay with more than one device — v1 rejects it with a clear error but
  keeps the `deviceId` plumbing so it is additive later.
- Multi-device nodes running *on* a device. In v1 such nodes (e.g. a panorama stitcher) are
  host nodes; v2 moves them onto an RVC4 fed by the relay from goal 2.
- ABI stability. `PipelineImpl` is defined in the public header with inline forwarders
  (`include/depthai/pipeline/Pipeline.hpp:44-290`, `:600-605`), so layout changes are ABI
  breaks today; we do not promise otherwise. We still append members and add new API
  out-of-line as hygiene.
- New tests or CI changes. Verification in this plan is by examples only.

---

## 2. Current state (what the code does today)

Everything below is single-device by construction; file references are to this branch.

| Area                | Today                                                                                                                                                                                | Ref                                                                                                |
| ------------------- | ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------ | -------------------------------------------------------------------------------------------------- |
| Device binding      | `PipelineImpl` holds one `std::shared_ptr<Device> defaultDevice`                                                                                                                     | `include/depthai/pipeline/Pipeline.hpp:184`                                                        |
| Node creation       | `createNode<N>()` always passes `defaultDevice`; `adoptSubtree` fills it in only when a node has none                                                                                | `Pipeline.hpp:224-248`, `src/pipeline/Pipeline.cpp:833-836`                                        |
| Per-node device     | `DeviceNode::device` + `get/setDevice()` and `DeviceNodeCRTP::create(device, …)` already exist                                                                                       | `include/depthai/pipeline/DeviceNode.hpp:14,32,56,91`                                              |
| Schema              | One schema; every device node stamped with the same `defaultDeviceId`; `NodeObjInfo::deviceId` already in the wire format                                                            | `Pipeline.cpp:410,550-577`, `include/depthai/pipeline/NodeObjInfo.hpp:17`                          |
| Bridges             | `build()` creates XLink bridges for device↔host only; all bound to `defaultDevice->getConnection()`; the device→device case exists only as a comment                                 | `Pipeline.cpp:1040-1134` (`:1043-1061` comment), `:1092,1121,1162`                                 |
| Start/stop          | `start()` calls `startPipeline` on one device; `stop()` closes one device                                                                                                            | `Pipeline.cpp:1207-1217,1336-1338`                                                                 |
| Reconnect           | `resetConnections()` rebinds **every** XLink host node to the default device                                                                                                         | `Pipeline.cpp:1256-1281`                                                                           |
| Failure             | any host node `runtime_error` → `stopPipeline()` → close device                                                                                                                      | `src/pipeline/ThreadedNode.cpp:50-58`                                                              |
| Cross-pipeline link | `Output::canConnect()` does not check `isSamePipeline()` — accepted by accident                                                                                                      | `src/pipeline/Node.cpp:107-148`                                                                    |
| Timestamps          | Firmware computes the host offset (host only echoes on `__timesync`), so `Buffer::ts` arrives in host `steady_clock` units; `tsDevice` is per-device monotonic                       | `src/device/DeviceBase.cpp:1308-1328`                                                              |
| Sync                | `Sync::run()` blocks on *all* inputs; `MessageGroup::isSynced()` hardcodes `getTimestampDevice()`                                                                                    | `src/pipeline/node/Sync.cpp:268-275`, `src/pipeline/datatype/MessageGroup.cpp:32-44`               |
| Memory              | `SharedMemory` is a bare fd with no origin; `XLinkOutHost` forwards any fd regardless of destination protocol; XLink silently mmaps+copies (and leaks the mmap) for non-shdmem links | `include/depthai/utility/SharedMemory.hpp:24`, `src/pipeline/node/internal/XLinkOutHost.cpp:72-81` |
| Local device        | XLink shdmem discovery reports name `/tmp/xlink.sock`, **empty deviceId**, **platform MYRIAD_X**; only found after the full search timeout                                           | XLink `local_memshd.cpp:225-247`, `DeviceBase.cpp:234,279`                                         |
| Protocol env        | `DEPTHAI_PROTOCOL` is a process-global discovery filter; `TCP_IP_OR_LOCAL_SHDMEM` connect tries the local socket first and discards the IP                                           | `src/xlink/XLinkConnection.cpp:94-115,198,269`, XLink `tcpip_memshd.cpp:117-128`                   |
| Python              | all node bindings funnel through one `pyNodeCreateMap` lambda calling `p.create<T>()`                                                                                                | `bindings/python/src/pipeline/node/Common.hpp:21-26`                                               |
| Multi-device today  | one `Pipeline` per `Device`, host `Sync`, manual queue pumping threads                                                                                                               | `examples/cpp/Misc/MultiDevice/multi_device_frame_sync.cpp:113-121,190-191`                        |

---

## 3. Object model and API

### 3.1 Master and the device set

- **Master** is `getDefaultDevice()`. It is (a) the device used for nodes created without an
  explicit device and (b) the implicit calibration/geometry reference for host nodes that
  today call `getParentPipeline().getDefaultDevice()` (RGBD, VIO/SLAM, beta parsers). It has
  **no other privileges**: it is not an FSYNC role and it is not a lifecycle anchor.
- Devices become part of the pipeline **implicitly on first use** (`create<N>(dev)`,
  `add(node)` with a pre-set device) or **explicitly** via `addDevice(...)`.
- `getDevices()` returns master first, then insertion order.
- `Pipeline(createImplicitDevice=false)` followed by an explicit device is allowed; the
  **first attached device is promoted to master**, so `getDefaultDevice()` and un-annotated
  nodes keep working.

```cpp
class Pipeline {
    // existing
    std::shared_ptr<Device> getDefaultDevice();

    // new
    std::shared_ptr<Device> addDevice(std::shared_ptr<Device> device);   // primitive
    std::shared_ptr<Device> addDevice(const DeviceInfo& info);            // constructs Device
    std::shared_ptr<Device> addDevice(const std::string& idOrIpOrName);   // constructs Device
    std::vector<std::shared_ptr<Device>> getDevices() const;              // [master, ...]

    template <class N, class... Args>
    std::shared_ptr<N> create(std::shared_ptr<Device> device, Args&&... args); // new overload
    template <class N, class... Args>
    std::shared_ptr<N> create(Args&&... args);                                 // unchanged
};
```

`create<N>(device, args...)` forwards to the already-existing
`DeviceNodeCRTP::create(device, args...)` and registers `device`. Creating a host node with a
device argument is a compile-time error. `Subnode<T>` keeps inheriting its parent's device.

### 3.2 Python

Only the `create()` path gains device selection:

```python
with dai.Pipeline(devA) as p:
    camA = p.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_A)
    camB = p.create(dai.node.Camera, device=devB).build(dai.CameraBoardSocket.CAM_A)
```

This is one change in `Common.hpp`'s `pyNodeCreateMap` lambda signature plus
`PipelineBindings.cpp` (`createNode` and the `create` binding); the ~50 per-node binding files
are untouched. Constructor-style creation inside `with dai.Pipeline()` keeps using the
master. Also bound: `addDevice`, `getDevices`, `DeviceNode.getDevice`, the device-state
API (§5) and `DeviceInfo.local()` (§8).

### 3.3 Per-device configuration

Every pipeline-level device setter/getter gains a `(device, ...)` overload; the existing
no-device form targets the master (zero behaviour change):

`setCameraTuningBlobPath`, `setXLinkChunkSize`, `setSippBufferSize`, `setSippDmaBufferSize`,
`isCalibrationDataAvailable`, `getCalibrationData`, `setCalibrationData`, `getEepromData`,
`setEepromData`, `getEepromId`, `setDefaultDeviceProperties`/`getDefaultDeviceProperties`
(as `setDeviceProperties(device, …)`), `getDeviceConfig(device)`, `setBoardConfig(device, …)`.
Board config becomes a per-device map internally; `board` remains the master's entry.

### 3.4 Links across devices

- `outA.link(inB)` where A and B are different devices is **accepted silently**; `build()`
  inserts a relay (§4.3) and logs one info line with the stream name and both device ids.
  Relays are visible in the schema (`bridges`) like other XLink bridges.
- Linking across two different `Pipeline` objects — accepted by accident today — becomes an
  error at `link()` time (`Output::canConnect()` gains the `isSamePipeline()` check).

### 3.5 Input → source device (no per-message identity)

Messages do **not** carry a device id. At `build()` the pipeline resolves, for every
`Node::Input`, the device that produces its data: the device of the upstream node, or
"host" if the upstream node runs on host (no walking through host pass-through nodes). This
is exposed as:

```cpp
std::shared_ptr<Device> Node::Input::getSourceDevice() const;   // valid after build()
std::map<std::string, std::shared_ptr<Device>> Node::InputMap::getSourceDevices() const;
```

A multi-device host node (e.g. a stitcher) uses this to learn which input is which device.
`Sync` uses it for validation (§6).

---

## 4. Backend design

### 4.1 `PipelineImpl` state

Appended members (append-only to limit ABI churn):

```cpp
std::vector<std::shared_ptr<Device>> devices;             // devices[0] == defaultDevice once set
std::unordered_map<Device*, DeviceRuntime> deviceRuntime; // bridges, relays, state, per-device board config
std::vector<std::shared_ptr<HostRelay>> relays;
std::function<void(std::shared_ptr<Device>, DeviceState)> deviceStateCallback;
```

`defaultDevice` stays as the master pointer so every existing inline forwarder keeps
working. Device nodes that today reach the device via
`getParentPipeline().getDefaultDevice()` and are themselves `DeviceNode`s (e.g.
`src/pipeline/node/Camera.cpp:291`, NN/DetectionNetwork platform checks, `BetaNode`) are
switched to their own `getDevice()` **(assumption: only host nodes keep the master as
reference)**. Mixed RVC2+RVC4 pipelines are therefore allowed without a hard check; each
node's platform logic reads its own device **(assumption)**.

### 4.2 Build: partition, validate, bridge

`PipelineImpl::build()` becomes:

1. Holistic record/replay setup: if `devices.size() > 1` and record/replay is enabled →
   throw `"Holistic record/replay is not supported with multiple devices yet"`.
2. AutoCalibration auto-insertion runs per device that has exactly one stereo pair.
3. `buildStage1/2/3` as today.
4. **Bridge insertion**, now three cases, each bound to the *owning* device's connection:
   - device → host: `XLinkOut`(dev) + `XLinkInHost`(dev connection)
   - host → device: `XLinkOutHost`(dev connection) + `XLinkIn`(dev)
   - device A → device B: `XLinkOut`(A) + `HostRelay`(A→B) + `XLinkIn`(B) — §4.3
   Stream names stay `__x_{nodeId}_{group}_{name}`; node ids are pipeline-unique so names are
   unique across devices. `uniqueStreamNames` becomes per-device for clarity.
5. Output-queue bridges per owning device.
6. Pipeline debugging: one `PipelineEventAggregation` **per device**, merged by a single
   host `PipelineStateMerge` **(assumption)**.
7. Resolve input→source-device map (§3.5) and the **fatal-device set** (§5.3).
8. Per-device schema: `getDevicePipelineSchema(deviceId)` keeps only nodes whose
   `deviceId` matches; assets are serialised per device. `NodeObjInfo::deviceId` is always
   populated with the real `DeviceInfo::getDeviceId()`.

All validation (schema, platform support of each node, cross-pipeline links, record/replay)
completes **before any device is touched**.

### 4.3 Host relay (device → device)

A single new internal host node `HostRelay : ThreadedHostNode` that owns *one* inbound
`XLinkStream` on device A's connection and *one* outbound stream on device B's connection;
one thread, one internal queue (default depth 8, non-blocking; both configurable via a
`getXLinkBridge()`-style handle). Rules:

- **Memory.** If the received message is fd-backed (`SharedMemory`) **and** the destination
  connection protocol is `X_LINK_LOCAL_SHDMEM`, forward the fd (zero-copy). Otherwise write
  the mapped bytes — exactly one copy in total, the same as today's receive memcpy. The XLink
  `writeFdEventMultipart` mmap leak is fixed in the XLink PR (§9).
- **Throttling.** The A-side `XLinkOut` is reachable through the existing
  `Output::getXLinkBridge()->xLinkOut->setFpsLimit()`.
- **Lifecycle.** A relay follows the state of *both* endpoints: if A dies it idles; if B dies
  it drops (non-blocking) until B is back.

This node is also the v2 path for an RVC4 that consumes other devices' streams: nothing in
the relay assumes an x86 host, and the fd rule already covers a local-shdmem destination.

### 4.4 Start / stop

- `start()`: `build()` (all validation) → set `pipelinePtr` on every device → call
  `startPipeline(schema_i)` on **all devices in parallel** → if any fails, stop the ones that
  started and rethrow (**all-or-nothing**) → start host nodes, relays and bridges. Master
  goes first only if a node on it declares an ordering need (none in v1).
- `stop()`: stop host nodes → close all devices in parallel → finish record/replay.
- `wait()` returns when the pipeline stops (§5.4).
- The process-wide `pipelineBuildMutex` is kept.

---

## 5. Partial operation and failure model

### 5.1 Device state

```cpp
enum class DeviceState { RUNNING, DISCONNECTED, RECONNECTING, FAILED };
DeviceState Pipeline::getDeviceState(std::shared_ptr<Device>) const;
void Pipeline::setDeviceStateCallback(std::function<void(std::shared_ptr<Device>, DeviceState)>);
```

State transitions are driven by the per-device monitor/watchdog already in `DeviceBase`.

### 5.2 What consumers observe

When device B dies, B's bridge and relay threads exit and **B's downstream inputs go idle** —
no close, no exception, no sentinel. Blocking `get()` callers must use timeouts or
`waitAny`. Host nodes are unaffected unless they react to the state callback.

### 5.3 Reconnect and fatality

- **Per-device auto-reconnect** with today's knobs (`setMaxReconnectionAttempts`,
  `DEPTHAI_RECONNECT_TIMEOUT`). `resetConnections()` becomes `resetConnections(device)`:
  only that device's bridges and relays are rebound and only its schema is re-sent. Other
  devices are untouched. After reconnect, B's streams resume with fresh sequence numbers and
  a `RUNNING` event fires.
- **Fatal devices.** A device is fatal iff a node running *on that device* consumes streams
  from other devices (i.e. it is the B side of a relay whose consumer is a device node).
  Derived at `build()`, no user flag. In v1 no such nodes exist, so **no device is fatal** —
  including the master.

### 5.4 When the pipeline stops on its own

1. a fatal device is lost for good (reconnect exhausted), or
2. a host node raises `std::runtime_error` (unchanged), or
3. **all** devices are gone with reconnect exhausted.

Otherwise the pipeline keeps running and `isRunning()` stays true.

---

## 6. Time and sync

- Cross-device comparison uses `Buffer::ts` (host domain, already written by firmware) or
  `tsSystem` (PTP); never `tsDevice`. Residual error is the difference of two independent
  timesync loops (measured ≈1 ms mean / 2 ms p99 on the fsync testbed).
- `Sync` (host): rewritten around `MessageQueue::waitAny` plus the device-state signal
  instead of blocking `get()` on every input, so a stalled or dead device cannot deadlock
  the node. While any input's device is not `RUNNING` the node **drops** (emits nothing);
  there is no partial-group mode in v1.
- If `Sync`'s inputs span more than one device (via `getSourceDevices()`),
  `TimestampSource::DEVICE` throws at `build()`; `DEFAULT` resolves to `HOST`.
- `MessageGroup::getIntervalNs()/isSynced()` become source-aware (they currently hardcode
  `getTimestampDevice()`); the group records which timestamp source produced it.
- Hardware sync: `CameraControl::setFrameSyncMode`, `ExternalFrameSyncRole` and the PTP flow
  are documented as-is; nothing is automated.

---

## 7. Memory

- **Boundary rule** (§4.3): fd forwarded only to a LOCAL_SHDMEM destination; otherwise one
  copy. `SharedMemory` ownership is unchanged (dtor closes the fd); relays never hand out raw
  fds.
- **No extra copies** beyond today's one receive memcpy on USB/TCP. Fan-out keeps sharing a
  single `shared_ptr<ADatatype>`; in-place `Buffer::setData` hazards are documented, not
  redesigned.
- **Bounded footprint**: no new global limits. The cost table is the sizing tool:

| Per …              | Cost                                                                     |
| ------------------ | ------------------------------------------------------------------------ |
| device             | 6 housekeeping threads (7 on RVC4 with the gate monitor)                 |
| host→device bridge | 1 thread + 5 MiB + 50 KiB XLink write buffer (resizable)                 |
| device→host bridge | 1 thread + receive memcpy per message (USB/TCP)                          |
| relay              | 1 thread + 1 queue (depth 8) + outbound write buffer                     |
| input              | queue depth 3 (blocking) by default; `Sync` inputs depth 10 non-blocking |
| output queue       | depth 16 non-blocking by default                                         |

Knobs exist only on the relay (depth, blocking, fps limit).

---

## 8. RVC4 standalone

- **Local device addressing.** XLink shdmem discovery is fixed to report platform RVC4 and
  the device's real id (read from the device), and the pinned XLink commit is bumped. Core
  adds `DeviceInfo::local()` (protocol `X_LINK_LOCAL_SHDMEM`, state booted) so
  `Device(DeviceInfo::local())` skips the search timeout, and `getAnyAvailableDevice()`
  includes `X_LINK_BOOTED` shdmem devices in its primary loop.
- **Remote devices.** `DeviceInfo(std::string ip)` always resolves to `X_LINK_TCP_IP`, never
  `TCP_IP_OR_LOCAL_SHDMEM`, so it can no longer silently land on the local socket. Local +
  remote in one process works with the default (`any`) protocol env; the README calls out
  that `DEPTHAI_PROTOCOL=shdmem|tcpip` and a non-empty `DEPTHAI_DEVICE_ID_LIST` are
  process-wide filters.
- **Topology is app code.** No new env var or config format; the app passes ids/IPs. The
  README section for standalone lists the requirements: outbound TCP to each remote's gate
  (11492) and XLink port, the RVC4 FWP embedded or reachable, and `/tmp` shared with the
  device runtime. App packaging (`oakctl`, images) lives outside this repo and is unchanged.
- Watchdog timeout selection (`== X_LINK_TCP_IP` only) is corrected to treat
  `X_LINK_LOCAL_SHDMEM` explicitly.
- Telemetry's `standalone` flag is set for shdmem connections, not just loopback names.

---

## 9. Implementation plan (ordered PRs)

Each PR is independently mergeable, keeps the existing test suite green, and is
source-compatible. Sizes are relative.

| #   | PR                                                         | Scope                                                                                                                                                                                                                                                                                                                 | Size |
| --- | ---------------------------------------------------------- | --------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- | ---- |
| 0   | **Design doc**                                             | this file                                                                                                                                                                                                                                                                                                             | S    |
| 1   | **XLink: shdmem identity + fd write leak** (luxonis/XLink) | report RVC4 platform and real deviceId for the local device; `munmap` in `writeFdEventMultipart`; fix last-byte drop in the shdmem read path. Bump pinned commit in `cmake/depthaiDependencies.cmake`.                                                                                                                | S    |
| 2   | **Core: local device + remote hardening**                  | `DeviceInfo::local()`, booted-shdmem in primary search loop, `DeviceInfo(ip)` → `TCP_IP`, watchdog protocol fix, telemetry `standalone`.                                                                                                                                                                              | S    |
| 3   | **Core: device registry + `create<N>(device, …)`**         | `devices`, `addDevice`, `getDevices`, master promotion, `create` overload, `DeviceNode` call sites switched to own `getDevice()`, cross-pipeline `link()` error. Python: `device=` kwarg, `addDevice`, `getDevices`, `DeviceNode.getDevice`.                                                                          | M    |
| 4   | **Core: per-device build/start/stop**                      | per-device schema + assets, bridges bound to owning device, parallel atomic start, parallel stop, per-device pipeline debugging aggregators, record/replay rejection, input→source-device map.                                                                                                                        | L    |
| 5   | **Core: host relay**                                       | `HostRelay` node, device→device bridge case in `build()`, fd/copy rule, knobs, schema visibility.                                                                                                                                                                                                                     | M    |
| 6   | **Core: partial operation**                                | `DeviceState` + callback, idle-on-loss semantics, `resetConnections(device)`, fatal-device set, auto-stop rules.                                                                                                                                                                                                      | L    |
| 7   | **Core: Sync hardening**                                   | `waitAny`-based `Sync`, drop-while-degraded, DEVICE-source rejection across devices, source-aware `MessageGroup::isSynced()`.                                                                                                                                                                                         | M    |
| 8   | **Core: per-device setters**                               | `(device, …)` overloads for every pipeline-level device setter/getter, per-device board config, Python bindings.                                                                                                                                                                                                      | M    |
| 9   | **Examples**                                               | rewrite `examples/{cpp,python}/Misc/MultiDevice/multi_device_frame_sync` on the new API (register the Python one); new `multi_device_host_node` (stitcher-style, uses `getSourceDevices()`, tolerates a dead device); new `device_to_device_relay` (Camera on A → ImageManip/NN on B). Registered, `enable_test=OFF`. | M    |
| 10  | **Docs**                                                   | README section for multi-device and standalone; Doxygen comments on all new API (they are the Python docstrings).                                                                                                                                                                                                     | S    |

Dependency order: 1 → 2 → 3 → 4 → {5, 6, 7, 8} → 9 → 10. PRs 5–8 are independent of each
other once 4 has landed.

---

## 10. Open items carried into implementation

- Exact names of the new API (`addDevice`, `getDevices`, `getDeviceState`,
  `setDeviceStateCallback`, `getSourceDevice`) are proposals; settle in PR 3/6 review.
- Which XLink fork branch receives PR 1 and its release cadence.
- Whether `HostRelay` should default to non-blocking/depth 8 or mirror existing bridge
  defaults.

---

## Appendix: v1 implementation notes (deviations from the plan)

The plan above was implemented on `feat/multi_device_support`. A base MVP (per-node
device assignment, per-device schema filter, per-device bridge binding, sequential
multi-device start/stop) had already landed via develop; the remaining work follows
this plan with the deviations below.

- **PR 1 (XLink)** is not part of this repo and remains open: shdmem discovery still
  reports MYRIAD_X/empty id, and the `writeFdEventMultipart` mmap leak still stands.
  `DeviceInfo::local()` works independent of that fix (booted-device re-search fills
  the name in).
- **Relay (§4.3)** is composed from the existing bridge nodes -
  `XLinkOut(A) → XLinkInHost(A) → XLinkOutHost(B) → XLinkIn(B)` - instead of one
  fused `HostRelay` node, so it reuses the battle-tested reconnect/rebind machinery.
  Two threads instead of one; the observable contract of §4.3 holds (fd rule via the
  destination protocol, non-blocking depth-8 relay queue, fps knob through
  `getXLinkBridge()`, schema visibility, one info log line).
- **Fatal devices (§5.3)**: with the relay implemented, cross-device consumers do
  exist in v1 - the B side of every relay is derived as fatal at build.
- **§4.2 step 6 (per-device pipeline-debugging aggregators)** is not implemented;
  multi-device pipelines keep rejecting pipeline debugging (pre-existing MVP throw).
- **§4.2 step 4** `uniqueStreamNames` stays one global set - node ids are
  pipeline-unique so stream names are already unique across devices.
- **Source-aware `MessageGroup` (§6)** records its timestamp source in a
  non-serialized field: the wire format is shared with device firmware, and groups
  received from a device are DEVICE-synced anyway.
- **Sync (§6)**: `TimestampSource::DEVICE` with inputs from more than one device
  throws at build; `DEFAULT` additionally throws when such a Sync runs on device
  (it would resolve to DEVICE there). On host, DEFAULT resolves to HOST as before.
- **Tests**: the plan said examples-only verification; host-only unit tests
  (`multi_device_pipeline_test`, ctest label `onhost` without `ci`) were added
  anyway as a regression guard - CI selection is unchanged.
- **Reconnect hardening** beyond the plan: the reconnect probe now waits for the
  *lost device's id* instead of any available device (in a multi-device pipeline the
  healthy devices satisfied the any-device gate instantly and one failed attempt
  aborted the loop), and single attempts survive exceptions.
