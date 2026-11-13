# Host runtime

Use for import/loading problems, Python/native crashes, callbacks, thread hangs,
and shutdown. Capture the failing process before changing its environment.

## Python and binary identity

First use the collector for offline context. When importing the selected SDK is
appropriate, use the reproduction's interpreter in a fresh process:

```bash
python3 -c 'import sys, depthai as dai; print(sys.executable); print(dai.__file__); print(dai.__version__)'
```

This imports the extension and can initialize SDK resources/transport; it does
not open a Device explicitly. Compare the path with the intended build or wheel.
Check local `depthai.py` files, virtual environments, `PYTHONPATH`, architecture,
Python ABI, and Debug/Release selection. Distribution metadata alone does not
identify the module that Python imports.

For C++, inspect loaded paths with GDB `info sharedlibrary`, LLDB `image list`,
or Visual Studio/WinDbg modules. Check runtime dependencies, loader search paths,
C++ runtime compatibility and missing optional components before rebuilding.
Use binary-inspection tools on known artifacts; do not execute an untrusted
binary merely to identify it.

## Capture the reproduction

```bash
DEPTHAI_LEVEL=debug python3 .agents/skills/depthai-debugger/scripts/capture_run.py \
  --output-dir /tmp/depthai-host-run -- python3 -u /path/to/repro.py
```

The helper writes combined stdout/stderr to `host.log` and records argv, cwd,
timestamps, child return code and termination reason in `manifest.json`.
It inherits the environment without adding SDK settings. Run the context
collector in the same environment to record relevant settings.

This helper is for noninteractive reproductions: stdin is closed and output goes
to the log file. Use direct terminal/debugger capture when input or a TTY is part
of the failure. Its manifest reports the immediate child's status separately from
`process_group_present` on POSIX. The latter is conservative: unreaped zombies
can keep a group present. Descendants that leave the group are outside that check;
inspect device state separately.

Prefer frame/iteration bounds and normal pipeline closure. Optional
`--timeout SECONDS --grace SECONDS` first requests interruption, then termination
if needed. Allow time for crash retrieval. Timeout/interruption remains reported
even if the child subsequently exits zero. On POSIX, escalation targets the new
process group even after its leader exits. On Windows, console interruption is
attempted; fallback termination may be abrupt and descendants are not guaranteed
to terminate. A stopped host does not prove device session cleanup; inspect that
before rerunning.

## Python exceptions and hangs

Capture the complete traceback and first exception, including callback errors.
For hangs, temporarily add `faulthandler.dump_traceback_later(..., repeat=True)`
and cancel it on cleanup. `python -X faulthandler` helps with fatal Python-process
signals. Supplement Python stacks with native stacks for C++ waits.

Check callbacks, blocking queue calls, GIL acquisition/release, Python object
lifetimes and NumPy buffer ownership. Compare the same minimal operation in C++
when it distinguishes binding behavior from SDK/device behavior; preserve frame
format, queues and firmware. Use pdb/debugpy or the user's IDE for Python logic.
On-device source/container mapping is covered in [RVC4](rvc4-runtime.md).

## Native crash or deadlock

Use matching symbols and preserve the executable and relevant libraries.

| Host | Launch/attach | Evidence |
| --- | --- | --- |
| Linux | `gdb --args /path/to/app args` or `gdb -p PID` | `thread apply all bt full`, `info threads`, `info sharedlibrary`, fault registers |
| macOS | `lldb -- /path/to/app args` or `lldb -p PID` | `thread backtrace all`, `image list`, faulting thread |
| Windows | Visual Studio native/mixed debugger or WinDbg, exact PID | All-thread stacks, exception, modules and matching PDBs |

Attachment pauses execution and may trigger watchdogs. Check
[controls](controls-and-compatibility.md) and restore altered settings.
For deadlocks, sample stacks more than once; waiting for input is not inherently
a bug. Trace producer, lock owner, callback and shutdown threads together.
Calling SDK functions inside a halted debugger can deadlock on paused owners.

Use exception breakpoints to locate the original throw when errors are wrapped.
Analyze a core/minidump with executable and libraries from that run. SDK Backward
signal handling can interact with debuggers and other handlers; disabling it is
a controlled comparison, not a default fix.

## Memory, races and lifecycle

Use user-supplied ASan/UBSan builds for invalid memory/undefined behavior and a
separate TSan configuration for races. Check `cmake/sanitizers`, CMake cache and
effective flags; `DEPTHAI_SANITIZE` alone does not describe every dependency.
Instrumented timing differs from production; vendor binaries can be opaque.

Select supported allocation/CPU tools (e.g. heaptrack/perf or macOS Instruments)
when logs/stacks leave the question unresolved. Distinguish RSS, allocator
retention, live allocations, queue growth and retained message references.
For shutdown hangs inspect joins, callbacks, queue cancellation, stream closure
and reconnection. Retain stacks, module maps, configuration and failing iteration;
detach/resume paused processes and remove temporary instrumentation afterward.

Sources: [bindings](../../../../bindings/python/README.md),
[initialization](../../../../src/utility/Initialization.cpp),
[options](../../../../cmake/depthaiOptions.cmake),
[queue bindings](../../../../bindings/python/src/MessageQueueBindings.cpp).
