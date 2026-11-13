# RVC4 runtime, Gate and OS

Use after identifying an RVC4 session/process, accelerator, camera-stack or OS
boundary. Locate the target and execution context first: workstation, SDK
container, device OS and application container are different environments.

## Gate session versus firmware connection

Gate discovers/manages firmware sessions and packages; firmware XLink carries
pipeline data/RPC. An accessible OS or healthy Gate does not prove firmware health.

From an authorized device shell, useful initial observations include:

```bash
cat /etc/os-release
uname -a
systemctl status depthai_gate.service --no-pager
journalctl -u depthai_gate.service --since '10 minutes ago' --no-pager
ps -eo pid,ppid,stat,etime,args
df -h
```

Discover actual service names and privileges on the installed image. Retain
timestamps and the first error. Inspect Gate status/version/session endpoints
through the transport supported by the running version; the SDK's
`DeviceGate.cpp` is the public source for HTTP/USB behavior. Use only read/query
operations during initial collection.

Correlate session ID, firmware version/checksum, package/extraction path, PID,
state, start/stop times, stdout and crash artifacts. A busy device can have a real
owner, orphaned process or stale session; identify which before cleanup. Do not
use broad process-name kills. Session deletion or a new run can remove evidence.

A CRASHED session can precede core/compression readiness. Preserve that session
and use bounded retrieval rather than reconnecting immediately. Continue in
[crash dumps](crash-dumps.md).

If private sources are available, inspect the running revision's
`python/depthai_gate/{session_manager,watchdog,fwp_manager}.py` and entrypoint.
Some README sections are design proposals; verify implemented routes,
supervision, archive contents and cleanup instead of assuming them.

## Firmware selection and debugger

For a user-built package:

```bash
DEPTHAI_DEVICE_RVC4_FWP=/path/to/preserved/depthai-device-rvc4-fwp.tar.xz \
DEPTHAI_PLATFORM=rvc4 DEPTHAI_LEVEL=debug python3 -u /path/to/repro.py
```

Confirm `Overriding device fwp:`, package hash and packaged executable identity.
A fresh executable alongside a stale archive still tests the old package.

With private firmware access, use its SDK/dev container, existing launch
configuration and debugging utilities. Inspect scripts before invocation:
some combine build/deploy/run and some core helpers select the current build.
Use the exact executable, separate `.debug` file, matching sysroot/shared
libraries and source mappings.

For an authorized attach to an existing selected PID, run the matching cross-GDB
on the workstation/SDK container and carry its protocol over SSH stdio:

```text
(gdb) file /path/to/matching/depthai-device-rvc4
(gdb) set sysroot /path/to/matching/sysroot
(gdb) target remote | ssh -T user@device gdbserver --attach - PID
```

Replace the SSH target and PID with the selected device/process. This opens no
GDB TCP listener. As the [GDB manual](https://sourceware.org/gdb/current/onlinedocs/gdb.html/Server.html)
explains, the hostname in a TCP endpoint such as `localhost:2345` is ignored by
gdbserver; that spelling does not restrict its listener to loopback.

Collect all-thread stacks, registers and module mappings. Detach explicitly when
finished to resume the attached process, and restore any supervisor changes.
Launching under gdbserver is a different experiment: preserve firmware environment,
working directory and library paths, and avoid a competing Gate-owned process.

Host watchdog/RPC settings, firmware watchdogs, Gate supervision and OS service
supervision are separate. `DEPTHAI_DEBUGGER` does not disable all of them.
If pausing the firmware requires pausing its supervisor, identify the exact
supervisor PID/service, account for other sessions, preserve state and explicitly
resume it afterward. Prefer the repository's supported debug launch when suitable.

## Compute and camera boundaries

| Layer | Evidence and methods |
| --- | --- |
| ARM process | All-thread stacks, CPU/allocation profiles, limits, FD/memory mappings; separate ASan/UBSan and TSan builds when supplied |
| GPU/OpenCL | Node stage timings, queue/event completion, upload/kernel/download or map costs, buffer ownership/coherency, effective clocks |
| DSP/Hexagon/FastRPC | Host-side RPC errors, library/runtime versions and paths, skel/stub pairing, DSP/OS logs, synchronization and returned buffers |
| NN runtime | Model compatibility, preprocessing/raw tensors, runtime selection and diagnostics from the installed QAIRT/SNPE/QNN components |
| Camera integration | Requested stream modes and socket/sensor mapping; frames and metadata before downstream processing |
| HAL/CHI/CamX | Usecase/topology selection, sensor/module definitions, tuning/3A, buffer flow, camera errors/dumps |
| Kernel/driver/sensor | Kernel messages, stream-start failures, IRQ/DMA/IOMMU evidence, device-tree configuration and sensor-control boundary |

Use vendor profilers/ramdumps only after checking installed SDK/tool versions,
target support and access. Do not invent a universal DSP debugger command.
CPU utilization alone cannot establish accelerator saturation. A backend timing
without valid output is not a successful computation.

For CamX/CHI, discover current OS repository skills and camera documentation.
Before changing log masks/buffer sizes or restarting camera services, preserve
existing logs and identify service ownership; Gate may manage camera-service
transitions. Enable targeted logs, reproduce once, capture topology/error output,
then restore settings. Never automatically clear logcat or allocate a large log
buffer from an old recipe.

## OS diagnostics

| Domain | Observations and next step |
| --- | --- |
| Process/service | Journal, restart history, PID/thread states, limits; distinguish process exit from whole-device reboot |
| Memory/storage | `free`, `df`, process RSS/maps, OOM/kernel messages, tmpfs and core/log growth; inspect the actual mount before deleting anything |
| Logs | `dmesg`, journal, available persistent/rotated logs, logcat where supported; preserve before clearing/reboot |
| Network/USB | Routes, interfaces/counters, negotiated speed, gadget/driver state, reset logs; correlate with host-side capture |
| Power/thermal | Profile, exposed supply negotiation, temperature, throttling and effective clocks; compare equal workloads |
| Time/synchronization | Clock domains, FSYNC/PTP and sensor timing, controller/driver evidence |
| Kernel tracing | Available tracepoints/ftrace/dynamic debug/perf; check kernel configuration, privileges and overhead before bounded capture |
| Boot/recovery | UART/boot logs, boot mode and active slot; use image/board-specific recovery procedure only within authorized scope |

Some images persist kernel/syslog data under `/overlay/persistent-logs`; discover
the installed configuration. Logcat persistence has changed across OS releases.
Do not assume `coredumpctl` owns Gate cores: inspect core_pattern, process limits
and the session's actual dump location.

Kernel modules, device-tree overlays, camera libraries, ABL and full images are
different deployment units. When a source fix is needed, locate the OS repository's
matching workflow and source-of-truth tree. Temporary Yocto workdir changes alone
are not a persistent fix. Respect the session's build/device-test constraints.

## Deployed applications

For oakctl/Hub applications, inspect installed `oakctl --help` and relevant
subcommand help; CLI naming varies by version. Collect app logs/state and identify
the actual container and Python/native process. App stdout, Gate logs and kernel
logs answer different questions. Use debugpy/IDE attachment with matching source
paths for Python logic. Restrict debugger exposure to the selected connection.

Public references: [DeviceGate](../../../../src/device/DeviceGate.cpp),
[SDK crash collection](../../../../src/device/CrashDumpManager.cpp),
[Luxonis OS](https://docs.luxonis.com/software-v3/sw-stack/luxonis-os),
[oakctl](https://docs.luxonis.com/software-v3/oak-apps/oakctl),
[app debugging](https://docs.luxonis.com/software-v3/oak-apps/debugging/).
Private references are discovered locally; do not copy proprietary implementation
or credentials into this public skill or investigation report.
