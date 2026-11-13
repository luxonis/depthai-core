# Crash dumps and live snapshots

Use for postmortems, missing dumps and stalled-device snapshots. Preserve the
original archive, reproduction log and exact binaries before rerunning or
rebuilding. A transport error, watchdog timeout and firmware fatal are different
observations; determine which occurred first.

## Capture

Use the local collection settings in
[controls-and-compatibility.md](controls-and-compatibility.md). Keep the host alive
long enough for reboot/retrieval or Gate compression; abrupt termination can lose
evidence. The SDK exposes `hasCrashDump()`, `getCrashDump(clearCrashDump=True)`
and `registerCrashdumpCallback(...)`; inspect binding/platform semantics.
Where supported, request without clearing when preservation is necessary.

The existing crash example deliberately crashes a device. Read it for callback
usage; do not run it as a passive collector.

RVC2-only `getState()` captures live state in crash-dump form, temporarily halting
execution. It replaces collected crash reports with the live snapshot and then
clears the device's cached crash evidence. Retrieve and persist any existing crash
dump before requesting a snapshot. This can disturb the pipeline and the API
recommends closing afterward. Select it deliberately for an authorized snapshot.

## Offline inspection

```bash
python3 .agents/skills/depthai-debugger/scripts/inspect_crashdump.py \
  /path/to/dump.tar --output-dir /tmp/depthai-dump \
  --symbol-file /path/to/preserved/symbols
```

The helper preserves the input, hashes supplied files, identifies JSON/SDK/Gate
archives or ELF cores, and extracts only recognized SDK payloads. It inventories
the nested RVC4 archive without extracting arbitrary files or executing a decoder.
It checks candidate `core.*` file headers: a filename alone does not establish
an ELF core. Header recognition is not full structural validation of a core.
It rejects traversal, links, duplicate members and over-budget archives.
`--max-bytes` limits both declared payload sizes and decompressed tar bytes read
across all layers, including metadata headers and padding. The decompressed read
limit applies before tarfile parses PAX/GNU extension headers. Large cores can
require a deliberate increase. Raw tar, gzip, bzip2 and xz are supported; JSON has
a separate small bound. Partial outputs on error are not a successful inspection.

Supplied symbol hashes establish identity only: `symbol_match` remains
`not_verified`. Compare build IDs, preserved build artifacts and firmware
identity before trusting symbolization.

After successful inspection, read a specific small log from the inventory without
extracting paths or executing content. Select the actual inner archive and member:

```python
import tarfile

with tarfile.open("/path/to/inspected/crash_dump.tar.gz") as archive:
    member = archive.getmember("exact-log-name-from-inventory")
    limit = 1024 * 1024
    if not member.isfile() or member.size > limit:
        raise ValueError("Expected a regular log no larger than 1 MiB")
    with archive.extractfile(member) as stream:
        print(stream.read(limit).decode("utf-8", errors="replace"))
```

For GDB, copy only the selected regular core member into an explicitly chosen new
file (exclusive creation), preserving the archive. Do not pass the outer SDK
archive to a decoder expecting a raw core. Read logs preceding the fatal signal:
`SIGABRT` alone does not establish the assertion or original cause.

| Input | Expected content / identity |
| --- | --- |
| Legacy RVC2 JSON | `depthaiCommitHash`, `deviceId`, `crashReports` |
| SDK RVC2 archive | `metadata.json`, `extra.json`, `crash_reports.json` |
| SDK RVC4 archive | `metadata.json`, `extra.json`, nested `crash_dump.tar.gz` |
| Gate archive | Core and process/system logs, contents vary by Gate/OS |
| Host or device ELF core | Determine architecture, process and mappings before choosing symbols |

Outer SDK metadata includes the **host** commit and compiled firmware pins.
Runtime overrides can differ. For RVC2, the inner crash report's firmware hash
and preserved MVCMD/ELF pair matter. For RVC4 use actual process/package identity
and mappings; do not use the current checkout's executable by convenience.

## RVC2 decoding

With authorized access to the RVC2 firmware repository, locate its crash reader
and inspect help/source. A common interface, from that repository root, is:

```bash
python3 utilities/crashDump/crash_dump_reader.py \
  --json /path/to/crash_reports.json --elf /path/to/matching/depthai.elf --verbose
```

The reader uses `addr2line`; ensure that executable supports the SPARC target.
Merely putting a prefixed cross-tool in PATH does not rename it to `addr2line`.
Check the exact toolchain and demangling dependency locally. Prefer preserved
local symbols; missing private download access should not prevent inspecting
assertion/trap information.

Start with processor, assertion/trap, crashed thread, instruction pointer and
call sites. Inspect all threads for cross-processor waits/deadlocks. Stack bounds
and frame pointers are data addresses, not necessarily symbolizable code.
A watchdog snapshot shows where execution was caught, not necessarily its cause.

## RVC4 / host native cores

For RVC4, use the matching AArch64 executable, its separate debug file, required
shared libraries and OS/SDK sysroot. Check ELF build IDs/debug links and source
mapping. Existing firmware utility scripts can help, but some select a fixed
current-build path; inspect them before invoking against an older core.

A GDB analysis session with user-selected matching paths:

```text
gdb-multiarch /path/to/matching/executable
(gdb) set sysroot /path/to/matching/sysroot
(gdb) set debug-file-directory /path/to/matching/debug-files
(gdb) core-file /path/to/core
(gdb) info sharedlibrary
(gdb) info threads
(gdb) thread apply all bt full
(gdb) info registers
```

The firmware SDK's cross-GDB may be preferable to a system GDB. Inspect warnings
about missing or mismatched files. Preserve raw addresses and module offsets if
symbols are unavailable. Host native cores/minidumps use the appropriate host
debugger and matching libraries/PDBs, described in [host runtime](host-runtime.md).

## When the dump is missing

Check these stages independently:

1. Did the device/process crash, lose transport, exit normally, or get killed?
2. Was collection disabled or the host stopped too soon?
3. For RVC2, did normal reboot/retrieval occur, or did a debugger intercept traps?
4. For RVC4, is the original Gate session still present and the artifact ready?
   A crashed state can precede core/compression readiness; retry within the
   collection deadline without replacing that session.
5. Is the Gate version/transport capable of retrieval?
6. Were core generation, process dumpability, resource limits, signal handling,
   destination permissions or disk/tmpfs space responsible?
7. Did retrieval succeed but callback/archive persistence fail? Was the saved
   path different from the assumed directory?

Retain both successful and failed stages in the report. Do not repeatedly force
crashes or erase sessions to test retrieval without that specific authorization.

Sources: [archive format](../../../../src/device/CrashDump.cpp),
[collection](../../../../src/device/CrashDumpManager.cpp),
[lifecycle](../../../../src/device/DeviceBase.cpp),
[Gate retrieval](../../../../src/device/DeviceGate.cpp),
[live-state contract](../../../../include/depthai/device/DeviceBase.hpp),
[callback example](../../../../examples/python/Misc/CrashDump/crash_dump.py).
