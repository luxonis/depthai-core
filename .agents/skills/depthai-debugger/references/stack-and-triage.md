# Stack and triage

Use when the failure layer or running build is unclear. Start with local files
and supplied logs; opening a Device can boot firmware or create a session.

## Identify what actually ran

Record the reproduction command, working directory, interpreter/executable,
SDK version and loaded path, host OS/architecture, target device ID, model,
platform, transport, sensor configuration, and failure phase. Also record:

- Host source commit and dirty state; build configuration and enabled components.
- Embedded firmware pins versus runtime override paths and file hashes.
- Firmware executable/package and matching symbols, including dirty changes.
- RVC4 OS/Gate version or RVC2 bootloader version when available.
- Model/archive hash, input data, pipeline schema, and relevant environment.
- Which values were measured, supplied by the user, or inferred from a checkout.

Run the local collector using the intended Python environment:

```bash
debug_evidence_dir="$(mktemp -d "${TMPDIR:-/tmp}/depthai-debug.XXXXXX")" || exit 1
python3 .agents/skills/depthai-debugger/scripts/collect_context.py \
  --repo . --build-dir build --output "$debug_evidence_dir/context.json"
```

Optional repeated `--related-repo /path/to/repo` and `--artifact /path/to/file`
add explicit repositories and SHA-256 identities. The collector does not import
depthai, discover devices, download symbols, or inspect arbitrary environment
variables. Its module candidate and installed-distribution version can disagree;
verify the real import separately in [host-runtime.md](host-runtime.md).
Paths, device selectors, and repository status can still be private.

A compiled SDK embeds firmware resources. Replacing a downloaded file does not
replace bytes already embedded in the library. Verify runtime overrides from
their logs and actual artifacts. A sibling checkout is not automatically the
dependency used by the SDK.

## Source map

Paths are relative to each repository. Locate private repositories through
user-provided paths, sibling checkouts, or existing workspace configuration.
Read their instructions before executing procedures. Scope searches to relevant
directories instead of entire vendor SDKs.

| Layer | Entry points | Boundary to inspect |
| --- | --- | --- |
| depthai-core application/API | `include/depthai`, `bindings/python/src`, matching `src` | Configuration, bindings, ownership, public contracts |
| Host pipeline | `src/pipeline/Pipeline.cpp`, `src/pipeline/node/internal`, `src/pipeline/node/host` | Node placement, generated bridges, queues, lifecycle |
| Device and RPC | `src/device/DeviceBase.cpp`, `src/device/DeviceGate.cpp` | Boot/session, RPC, watchdog, logging, reconnect |
| Wire representation | `include/depthai/{common,properties,pipeline/datatype}`, `src/utility/Serialization.cpp`, `protos` | Host/firmware definition compatibility |
| Firmware selection | `src/utility/Resources.cpp`, `cmake/Depthai` | Embedded pins versus runtime files |
| XLink | SDK `src/xlink`; XLink `src/shared`, `src/pc/protocols` | Discovery, streams, dispatcher, transport, packet release |
| RVC2 firmware | `depthai/device_cnn`, `utilities/crashDump` | LEON/RTEMS/SHAVE execution and firmware nodes |
| RVC4 firmware | `src/main.cpp`, `src/pipeline`, `src/camera`, `src/hexagon` | Linux process, camera integration, GPU/DSP/runtime |
| Gate | `python/depthai_gate` | Package/session lifecycle, process supervision, logs/dumps |
| RVC4 OS | OS documentation, Yocto layer, kernel, camera/vendor subrepos | Services, HAL/CamX/CHI, drivers, power, boot |
| RVC2 bootloader | Separate bootloader repo and SDK `DeviceBootloader` | Boot state, board config, persistent flashing |

Both firmware repositories consume SDK sources/definitions. Record their SDK
dependency revisions too. RVC4 applications can run on the device itself;
determine process location and actual protocol before assuming a network hop.
Gate controls the firmware session; distinguish that control connection from the
firmware's XLink data/RPC connection.

## Choose a useful comparison

| Observation | Next discriminating experiment |
| --- | --- |
| Failure before device access | Isolate import/library loading or host-only construction |
| Device visible to OS but not SDK | Compare permissions, protocol, filters, state and actual SDK build |
| Pipeline starts but one output stops | Sample queue/state/sequence progression and downstream consumption |
| XLink error follows device fatal log | Preserve crash/session evidence before transport retries |
| Wrong output with normal throughput | Compare intermediate payload and metadata with controlled input |
| Only long runs fail | Measure resource slopes, drops and temperature over equal-duration baselines |
| Only live cameras fail | Compare injected frames; retain sensor/timing/driver hypotheses |
| Only one OS/firmware combination fails | Compare one compatible component at a time with exact artifacts |
| Bug disappears under debugger | Compare watchdog, scheduling, optimization and instrumentation changes |

Without hardware, inspect logs, schema, crashes, source, and replayable inputs.
Without private access, provide the failing public API, boundary evidence,
required symbols/logs, and a precise handoff.

## Sources and freshness

SDK controls and limitations were checked against core commit `a8f707a54`.
This is a baseline, not a minimum version. Recheck the linked implementation and
tests at another revision. Private tool locations are discovery hints;
availability and hardware behavior require verification in that environment.

Public anchors: [DeviceBase](../../../../src/device/DeviceBase.cpp),
[Pipeline](../../../../src/pipeline/Pipeline.cpp),
[resources](../../../../src/utility/Resources.cpp),
[firmware pins](../../../../cmake/Depthai),
[XLink dependency](../../../../cmake/depthaiDependencies.cmake).
