# RVC2 firmware

Use when SDK/transport evidence points to Myriad X execution. Public SDK
collection works without private source access. Processor-level decoding and
debugger procedures need the corresponding firmware repository/toolchain.

## Preserve and select the image

Preserve the exact MVCMD and ELF together with map/symbol files, source revisions,
local changes and build flags. A generated commit string can be stale; it is not
proof that two artifacts match.

For an already built image and an explicitly selected target:

```bash
DEPTHAI_DEVICE_BINARY=/path/to/preserved/depthai.mvcmd \
DEPTHAI_PLATFORM=rvc2 DEPTHAI_LEVEL=debug python3 -u /path/to/repro.py
```

Confirm `Overriding firmware:` and a fresh device boot. This selects a session
image; persistent flashing is a separate operation. Firmware-only edits can be
tested with a runtime override; shared SDK/wire changes may need paired builds.

## Narrow the firmware boundary

Map the failing host RPC, node or stream into the device implementation.
Common source areas in a repository containing `depthai/device_cnn`:

| Area | Inspect |
| --- | --- |
| `leon` | Initialization, RPC, board/watchdog and host-facing operations |
| `leon_rt` | Real-time processor initialization and cross-processor behavior |
| `pipeline_builder` | Node implementations, plugins, pipeline allocation/config |
| `shared` | Shared firmware behavior |
| MDK components | RTEMS, drivers, transport, allocators and crash capture |

Follow the repository's own layout if it differs. Scope searches before entering
the large MDK tree. Compare device SDK dependencies with host definitions.

Start with logs and resource telemetry: LEON CPU/heaps, DDR, CMX, pools and SHAVE
allocation. Use deterministic input to isolate sensor acquisition. For a memory
or scheduling problem, trace retained buffers, task state, priorities, stack
bounds, wait owners and processor communication. Do not assume a blocked task's
top stack frame caused the fault.

Use [crash decoding](crash-dumps.md) for assertions, traps and live state.
Targeted firmware instrumentation should expose the suspected transition or
allocation, with a normal-settings comparison afterward. Have the user supply
builds when required by the session.

## USB/JTAG debugging

First discover the private repository's development skill or README and inspect
the actual debugger scripts/targets. Typical targets include `start_usb_server`,
`debug`, `debug_attach`, and board/network variants; the wrapper may start a
server, reset/load the image, or build prerequisites. Do not run make/debug
targets automatically when builds are prohibited.

Check probe/board support, device access rules, transport wiring, toolchain and
matching ELF. Distinguish reset-and-load from attach-to-running. Some USB debug
workflows require both USB2 and USB3 connectivity. Console endpoints, processor
selection, instruction-history and trap commands are toolchain-specific: obtain
them from the selected scripts rather than guessing addresses or ports.

Debugger scripts can disable watchdog/reset behavior, change debugger-context
flags, halt on traps before RTEMS crash handling, and change processor polling.
Record these effects. A trap intercepted by the debugger may never produce the
ordinary SDK crash report. Reduced optimization changes layout/timing too.

If the failure disappears under debugging, boot the preserved MVCMD normally and
compare logs/dumps. On completion detach/resume or reset only the selected target
as authorized, restore normal recovery settings, and verify ordinary boot.

## Bootloader and hardware escalation

If application firmware never starts, examine boot state, bootloader version,
board configuration, transport enumeration and boot logs before node code.
For camera/driver failures, separate sensor control, acquisition, memory/DMA and
processing using the smallest supported reproduction.

JTAG, UART and hardware measurements can localize failures inaccessible through
XLink. Use board-specific wiring/probe documentation and permitted readouts.
Register writes, EEPROM operations and recovery flashing need a concrete scoped
procedure; they are not generic troubleshooting steps.

Without private access, retain the original dump, public SDK log/schema, firmware
identity, target configuration and exact failure phase. Request matching symbols
or a specific processor/task observation rather than the entire firmware tree.

Sources: public [resource override](../../../../src/utility/Resources.cpp),
[RVC2 crash representation](../../../../include/depthai/device/CrashDump.hpp),
[resource APIs](../../../../include/depthai/device/DeviceBase.hpp).
Private discovery: RVC2 repository development skill, debugger targets/scripts,
and `utilities/crashDump`; validate them against the selected firmware revision.
