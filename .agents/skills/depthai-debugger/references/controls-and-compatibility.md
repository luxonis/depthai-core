# Controls and compatibility

Use the controls implemented in the running revision. The baseline here is
`a8f707a54`; source and tests take precedence over older descriptions for factual
behavior. Session/repository instructions still govern what may be executed.
Environment values are cached in parts of the SDK: set them before import or
initialization and use a fresh process for comparisons.

## Logging, selection and firmware

| Control | Meaning and evidence |
| --- | --- |
| `DEPTHAI_LEVEL=debug` | SDK initialization/schema/lifecycle logs. `trace` is more intrusive and can include message/state output. |
| `device.setLogLevel(...)` | Filters device messages sent to host. |
| `device.setLogOutputLevel(...)` | Filters printing on host. Use callbacks to capture independently of printing. Startup already occurred before setters run. |
| `XLINK_LEVEL` | Transport log level: debug/info/warn/error/fatal/off in the checked initialization code. |
| `DEPTHAI_DEBUG` | Numeric flag for additional device prints; can also lower host logging to debug. Distinct from debugger mode. |
| `DEPTHAI_PIPELINE_DEBUGGING=1` | Instrument pipeline; automatic state printing uses `DEPTHAI_LEVEL=trace`. Programmatic snapshots do not require trace. |
| `DEPTHAI_PROFILING=1` | Transfer counters/logging; use SDK debug level to see output. |
| `DEPTHAI_PLATFORM` | Restrict discovery (any/rvc2/rvc3/rvc4; inspect aliases in source). Does not convert a device to another platform. |
| `DEPTHAI_PROTOCOL` | any/usb/tcpip/tcpshd/shdmem at baseline; support depends on platform/build. |
| `DEPTHAI_DEVICE_ID_LIST`, `DEPTHAI_DEVICE_MXID_LIST`, `DEPTHAI_DEVICE_NAME_LIST` | Comma-separated selectors; check combined filter semantics and explicitly select the intended target. |
| `DEPTHAI_DEVICE_BINARY` | RVC2 runtime file override; verify `Overriding firmware:` and hash the file. |
| `DEPTHAI_DEVICE_RVC4_FWP` | RVC4 package override; verify `Overriding device fwp:` and package/executable freshness. |
| `DEPTHAI_BOOTLOADER_BINARY_USB` / `_ETH` | Bootloader resource overrides, not ordinary firmware debugging controls. |
| `DEPTHAI_RECORD` / `DEPTHAI_REPLAY` | Holistic record/replay locations; inspect topology/format support before use. |

## Timeouts and debugger effects

| Control | Scope / interpretation |
| --- | --- |
| `DEPTHAI_SEARCH_TIMEOUT` | Discovery wait, milliseconds |
| `DEPTHAI_BOOTUP_TIMEOUT` | Waiting for boot after image transfer, milliseconds |
| `DEPTHAI_CONNECT_TIMEOUT` | Establishing the connection, milliseconds |
| `DEPTHAI_RECONNECT_TIMEOUT` | Recovery after connection loss, milliseconds; zero disables reconnect |
| `DEPTHAI_RPC_READ_TIMEOUT` / `_WRITE_TIMEOUT` | RPC waits, milliseconds; inspect defaults and zero semantics in the selected code |
| `DEPTHAI_WATCHDOG` / `_INITIAL_DELAY` | Device watchdog timeout/initial delay, milliseconds; zero timeout disables normal recovery and can require reset after an unclean exit |
| `DEPTHAI_DEBUGGER=1` | When not explicitly overridden, host sets watchdog timeout and default RPC read timeout to zero; does not launch GDB or disable every supervisor |
| `DEPTHAI_INSTALL_SIGNAL_HANDLER=0` | Prevents installation of the SDK Backward handler where compiled/enabled; does not control every linked library's handlers |

Timeouts belong to different phases. Record effective settings and elapsed time
before increasing one. A scoped RPC timeout may take precedence over the
environment. Gate supervision and OS/service watchdogs are separate from host
debugger mode. Restore normal settings for final verification.

## Crash collection versus upload

At baseline:

- `DEPTHAI_CRASHDUMP=/existing/directory` selects local archive storage.
  Create the directory before starting. Otherwise the SDK uses its cache path;
  use the actual saved-path log.
- `DEPTHAI_CRASHDUMP=0` disables automatic collection.
- `DEPTHAI_CRASHDUMP_TIMEOUT` is in milliseconds; a nonpositive value disables
  automatic collection. Positive values bound retrieval.
- `DEPTHAI_DISABLE_CRASHDUMP_COLLECTION=1` suppresses upload in
  `LogCollection.cpp` **after** local archive creation. Despite its name, it
  preserves local capture in this revision.
- Any nonempty `DEPTHAI_DISABLE_FEEDBACK` suppresses legacy crash upload.
- `DEPTHAI_TELEMETRY=0` disables telemetry; it is a separate control.

For an authorized reproduction with local evidence only:

```bash
mkdir -p /tmp/depthai-crash-evidence
DEPTHAI_LEVEL=debug DEPTHAI_CRASHDUMP=/tmp/depthai-crash-evidence \
DEPTHAI_TELEMETRY=0 DEPTHAI_DISABLE_FEEDBACK=1 \
python3 -u /path/to/repro.py
```

Recheck semantics for another SDK. Capture, retrieval, callback delivery, local
archive creation and upload can fail independently. `DEPTHAI_CRASH_DEVICE=1`
enables deliberate fault injection; it is not needed to collect real crashes.

## Controls that change persistent state

Do not enable factory-flashing switches during routine diagnosis.
`DEPTHAI_AUTOCALIBRATION` can flash successful calibration to EEPROM by default.
Inspect supported modes and runtime/persistent behavior before using it.
Record existing configuration and restore only changes the investigation made.

Sources: [logging](../../../../src/utility/Logging.cpp),
[environment caching](../../../../src/utility/Environment.hpp),
[initialization](../../../../src/utility/Initialization.cpp),
[DeviceBase](../../../../src/device/DeviceBase.cpp),
[crash persistence/upload](../../../../src/utility/LogCollection.cpp),
[selection](../../../../src/xlink/XLinkConnection.cpp),
[resources](../../../../src/utility/Resources.cpp).
