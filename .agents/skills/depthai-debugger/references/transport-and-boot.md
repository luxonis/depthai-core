# Transport and boot

Use for discovery/boot failures, busy devices, disconnects and throughput limits.
First identify platform, actual protocol and failure phase; an XLink exception
may follow a crash or blocked pipeline.

## Discovery and target selection

Inspect device enumeration without constructing an arbitrary Device:

```python
import depthai as dai
for info in dai.Device.getAllAvailableDevices():
    print(info)
```

Use `getAllConnectedDevices()` when investigating states excluded by availability
filtering; check the selected API's semantics. Discovery can probe USB/network.
Pin the chosen target in the reproduction, for example
`dai.Device(dai.DeviceInfo(selected_device))`. Device creation can boot firmware.
Record name/ID, platform, protocol and state before and after the failed attempt.

Check environment filters together. Entries within a list are alternatives, but
different ID/name filters restrict results together. A stale filter can hide a
reachable camera. Explicit network selection can help distinguish broadcast
discovery from address reachability.

| Symptom | Checks before retrying |
| --- | --- |
| No OS enumeration | Cable, power, port/hub, device boot mode, OS hardware logs |
| OS sees it; SDK does not | Permissions/udev or Windows driver, USB support in SDK, container device access, filters, busy state |
| Found but boot fails | RVC2 image/bootloader or RVC4 Gate/package/session, exact override, boot logs and deadline |
| Opens then drops | First firmware/host error, watchdogs, resets, USB/network logs, power/thermal context |
| Busy/already in use | Application owner, actual PID/session and state; distinguish live owner, orphan and stale state |
| Only sustained traffic fails | Negotiated speed, counters, host consumption, flow control, allocations and transport captures |

Do not kill all matching processes, delete sessions, or reset unrelated devices.
Preserve logs/dumps before authorized recovery. Repeated blind reconnects can
replace the evidence-bearing session.

## USB

On Linux inspect `lsusb -t` and relevant kernel messages to establish topology,
negotiated speed, resets and disconnects. Check access to the actual USB node,
container mappings and driver ownership. On Windows inspect Device Manager and
driver identity; on macOS use system USB information and relevant OS logs.

For protocol-level evidence, use usbmon/Wireshark on supported Linux hosts,
USBPcap on Windows, or available platform tooling. Select the target bus/device,
bound capture duration/size and review payloads before sharing.

Compare one cable/port/hub/host/power variable at a time. A successful USB2 run
narrows failure conditions but does not prove a single root cause. Restore speed,
chunk-size or rate-limit changes after the experiment.

## Ethernet and local communication

Inspect route/interface selection, link speed, loss/error counters, addressing,
subnet/discovery, firewall and connection state. Compare explicit address access
with discovery. Use a bounded tcpdump/Wireshark capture when the failure needs
packet evidence. Check SDK/Gate source for actual ports/protocols instead of
assuming every connection uses the same socket.

For local/shared-memory protocols, inspect process placement, socket/FD ownership,
permissions and cleanup; network troubleshooting may be irrelevant. Record
whether the implementation chose TCP or a local path.

## XLink internals

Start with `XLINK_LEVEL=debug` and SDK debug logs on a focused reproduction.
Trace stream create/open, writes, reads, release/acknowledgement, cancellation
and close. Compare both ends when a request has no expected response.
Check packet ownership, available stream buffers, dispatcher waits and shutdown
cancellation. A read timeout can originate from an upstream node that produced
no data.

Use the pinned XLink sources from `cmake/depthaiDependencies.cmake`; a separate
checkout is not automatically used. `DEPTHAI_XLINK_LOCAL` is the CMake integration
point for a user-built local variant. Record libusb selection and actual dependency
commit. Existing XLink throughput, RTT, stream-opening and discovery-thread tests
can isolate transport behavior from pipeline behavior; run only available built
tests on the chosen target.

## Boot and recovery

RVC2 can pass through unbooted, bootloader and firmware states. Distinguish booting
a session image from persistent application/bootloader flashing. Read current
bootloader identity/configuration before changing it.

On RVC4 distinguish Gate/package/process startup from OS boot and camera
initialization. SSH access alone does not prove Gate or firmware health.
Continue in [RVC4 runtime](rvc4-runtime.md) for service and recovery evidence.

Bootloader replacement, EEPROM changes, power cycling and OS recovery require
the intended scope and target. Use the matching platform procedure with backed-up
configuration and recoverable artifacts; do not turn a connection error into an
automatic flash operation.

Sources: [discovery/connection](../../../../src/xlink/XLinkConnection.cpp),
[stream implementation](../../../../src/xlink/XLinkStream.cpp),
[initialization](../../../../src/utility/Initialization.cpp),
[DeviceGate](../../../../src/device/DeviceGate.cpp),
[XLink tests](../../../../tests/src/ondevice_tests/xlink_test.cpp),
[bootloader API](../../../../include/depthai/device/DeviceBootloader.hpp).
