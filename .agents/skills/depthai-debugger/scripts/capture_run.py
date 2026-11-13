#!/usr/bin/env python3
"""Capture an explicit noninteractive reproduction without changing its SDK environment."""

import argparse
from datetime import datetime, timezone
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import sys
import time


def stop_process(process, grace):
    """Interrupt our child/group, escalating only if it does not finish."""
    actions = []
    for phase, wait in (("interrupt", grace), ("terminate", 5), ("kill", 5)):
        if process.poll() is not None:
            break
        try:
            if os.name == "posix":
                sig = {"interrupt": signal.SIGINT, "terminate": signal.SIGTERM, "kill": signal.SIGKILL}[phase]
                os.killpg(process.pid, sig)
            elif phase == "interrupt":
                process.send_signal(signal.CTRL_BREAK_EVENT)
            elif phase == "terminate":
                process.terminate()
            else:
                process.kill()
            actions.append(phase)
        except ProcessLookupError:
            break
        except OSError as error:
            actions.append(phase + ": " + str(error))
        try:
            process.wait(timeout=wait)
            break
        except subprocess.TimeoutExpired:
            continue
    return actions


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output-dir", type=Path, required=True, help="New directory; parent must exist")
    parser.add_argument("--timeout", type=float, help="Optional runtime limit in seconds")
    parser.add_argument("--grace", type=float, default=60, help="Seconds allowed after interruption (default: 60)")
    parser.add_argument("command", nargs=argparse.REMAINDER, help="Executable and arguments after --")
    args = parser.parse_args()
    command = args.command[1:] if args.command[:1] == ["--"] else args.command
    if not command:
        parser.error("supply a reproduction command after --")
    if not math.isfinite(args.grace) or args.grace <= 0:
        parser.error("--grace must be finite and positive")
    if args.timeout is not None and (not math.isfinite(args.timeout) or args.timeout <= 0):
        parser.error("--timeout must be finite and positive")
    try:
        args.output_dir.mkdir(mode=0o700)
    except OSError as error:
        parser.error(str(error))

    started = time.monotonic()
    manifest = {
        "schema_version": 1, "command": command, "cwd": str(Path.cwd()),
        "started_utc": datetime.now(timezone.utc).isoformat(),
        "timeout_seconds": args.timeout, "grace_seconds": args.grace,
        "status": "starting", "returncode": None, "termination_actions": [],
    }
    manifest_path = args.output_dir / "manifest.json"
    manifest_path.write_text(json.dumps(manifest, indent=2) + "\n", encoding="utf-8")
    options = {"start_new_session": True} if os.name == "posix" else {
        "creationflags": subprocess.CREATE_NEW_PROCESS_GROUP
    }
    process = None
    result = 127
    try:
        with (args.output_dir / "host.log").open("wb") as log:
            process = subprocess.Popen(
                command, stdout=log, stderr=subprocess.STDOUT,
                stdin=subprocess.DEVNULL, **options
            )
            manifest.update(status="running", pid=process.pid)
            manifest_path.write_text(json.dumps(manifest, indent=2) + "\n", encoding="utf-8")
            try:
                result = process.wait(timeout=args.timeout)
                manifest["status"] = "exited"
            except subprocess.TimeoutExpired:
                manifest["status"] = "timed_out"
                manifest["termination_actions"] = stop_process(process, args.grace)
                result = 124
            except KeyboardInterrupt:
                manifest["status"] = "interrupted"
                manifest["termination_actions"] = stop_process(process, args.grace)
                result = 130
    except OSError as error:
        manifest.update(status="launch_or_capture_failed", error=str(error))
        if process is not None:
            manifest["termination_actions"] = stop_process(process, args.grace)
    finally:
        manifest["returncode"] = process.poll() if process is not None else None
        manifest["child_still_running"] = process is not None and process.poll() is None
        manifest["ended_utc"] = datetime.now(timezone.utc).isoformat()
        manifest["elapsed_seconds"] = time.monotonic() - started
        manifest_path.write_text(json.dumps(manifest, indent=2) + "\n", encoding="utf-8")
    print(args.output_dir)
    if result < 0:
        return min(255, 128 - result)
    return result if result <= 255 else 1


if __name__ == "__main__":
    sys.exit(main())
