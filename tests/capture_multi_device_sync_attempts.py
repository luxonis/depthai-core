#!/usr/bin/env python3
"""Run a user-selected sync test and preserve camera PID slots after every attempt.

Uses only the Python standard library and an external adb client. See SyncDebug.md.
Device access is limited to enumeration, directory inventory, stat and file reads.
"""

import argparse
import csv
import datetime
import hashlib
import json
import math
import os
from pathlib import Path
import re
import selectors
import shlex
import shutil
import signal
import subprocess
import sys
import tarfile
import threading
import time
from urllib.parse import quote
import uuid


REMOTE_DIR = "/tmp/depthai-camera-debug"
CAMERA_FILE = re.compile(r"camera-(\d+)-slot-(\d+)-(latest|stall-first|stall|fatal)\.json$")
DEVICE_FIELDS = (
    "schema", "pid", "process_session", "session", "camera_id", "socket", "sensor",
    "fps", "hostname", "steady_start_ns", "system_start_ns", "anchor_steady_end_ns",
    "captured_steady_ns", "reason", "stop_requested",
)
# All remote operations are read-only. Shell quoting is applied to every argument.
INVENTORY_SH = r'''
if ! test -d "$1"; then
    status=3
    if test -e "$1" || ! test -x "${1%/*}"; then status=5; fi
    printf 'CAPTURE_INVENTORY_STATUS|%s\n' "$status"
    exit 0
fi
test -r "$1" && test -x "$1" || { printf 'CAPTURE_INVENTORY_STATUS|5\n'; exit 0; }
status=0
for p in "$1"/camera-*-slot-*.json; do
    test -f "$p" || continue
    stat -c '%n|%i|%s|%Y|%Z' -- "$p" || status=4
done
printf 'CAPTURE_INVENTORY_STATUS|%s\n' "$status"
exit "$status"
'''
READ_SH = r'''
printf '%s\n' "$2"
stat -c '%i|%s|%Y|%Z' -- "$1"
before=$?
printf '%s\n' "$3"
cat -- "$1"
read_status=$?
printf '\n%s\n' "$4"
stat -c '%i|%s|%Y|%Z' -- "$1"
after=$?
printf '%s|%s|%s\n' "$before" "$read_status" "$after"
test "$before" -eq 0 && test "$read_status" -eq 0 && test "$after" -eq 0
'''


def utc_now():
    return datetime.datetime.now(datetime.timezone.utc).isoformat(timespec="microseconds")


def utc_name():
    return datetime.datetime.now(datetime.timezone.utc).strftime("%Y%m%dT%H%M%S.%fZ")


def digest(data):
    return hashlib.sha256(data).hexdigest()


def serial_component(serial):
    component = quote(serial, safe="-_.")
    return component if component not in ("", ".", "..") else "serial-" + component.replace(".", "%2E")


def write_json(path, value):
    # Only runner-owned progress metadata is replaced; raw snapshots use exclusive writes.
    temporary = path.with_suffix(path.suffix + ".tmp")
    with temporary.open("x", encoding="utf-8") as output:
        json.dump(value, output, indent=2, ensure_ascii=True)
        output.write("\n")
        output.flush()
        os.fsync(output.fileno())
    temporary.replace(path)


def positive_int(value):
    result = int(value)
    if result <= 0:
        raise argparse.ArgumentTypeError("must be positive")
    return result


def positive_float(value):
    result = float(value)
    if not math.isfinite(result) or result <= 0:
        raise argparse.ArgumentTypeError("must be finite and positive")
    return result


def poll_interval(value):
    result = float(value)
    if not math.isfinite(result) or result < 0 or 0 < result < 1:
        raise argparse.ArgumentTypeError("use 0 (disabled) or at least 1 second")
    return result


def local_stat(path):
    info = path.stat()
    return {"inode": info.st_ino, "size": info.st_size, "mtime_ns": info.st_mtime_ns,
            "ctime_ns": info.st_ctime_ns}


def remote_stat(text):
    inode, size, mtime, ctime = map(int, text.strip().split("|"))
    return {"inode": inode, "size": size, "mtime_seconds": mtime, "ctime_seconds": ctime}


def json_object(data):
    value = json.loads(data)
    if not isinstance(value, dict):
        raise ValueError("expected a JSON object")
    return value


class Capture:
    def __init__(self, args, directory, metadata, adb):
        self.args = args
        self.directory = directory
        self.metadata = metadata
        self.adb = adb
        self.baseline = {}
        self.last_inventory = {}
        self.inventory_complete = {}
        self.saved = {}
        self.device_frames = {}
        self.poll_copies = 0
        self.poll_bytes = 0
        self.poll_limit_reported = False
        self.stop = threading.Event()
        self.worker = None
        self.manifest = (directory / "collection.jsonl").open("x", encoding="utf-8")

    def event(self, kind, **fields):
        entry = {"kind": kind, "observed_utc": utc_now(), **fields}
        self.manifest.write(json.dumps(entry, ensure_ascii=True) + "\n")
        self.manifest.flush()
        return entry

    def error(self, operation, **fields):
        entry = self.event("collection_error", operation=operation, **fields)
        self.metadata["collection_errors"].append(entry)
        print("Capture error: " + json.dumps(entry, ensure_ascii=True), file=sys.stderr, flush=True)

    def adb_call(self, arguments, phase, serial=None, deadline=None):
        if self.adb is None:
            return None
        remaining = self.args.adb_timeout
        if deadline is not None:
            remaining = min(remaining, deadline - time.monotonic())
        if remaining <= 0:
            self.error("collection_deadline", phase=phase, serial=serial)
            return None
        command = [self.adb, *arguments]
        started = utc_now()
        try:
            result = subprocess.run(command, stdout=subprocess.PIPE, stderr=subprocess.PIPE,
                                    timeout=remaining, check=False)
        except subprocess.TimeoutExpired as exc:
            partial = {}
            if exc.stdout and "exec-out" in arguments:
                relative, sha = self.save_bytes(exc.stdout, self.directory / "device" / serial_component(serial)
                                                / "read-errors" / ("timeout-" + uuid.uuid4().hex + ".read-output"))
                partial = {"destination": relative, "sha256": sha, "completed_json": False}
            self.error("adb_timeout", phase=phase, serial=serial, command=command,
                       timeout_seconds=remaining, returncode=None,
                       received_bytes=len(exc.stdout or b""),
                       received_sha256=digest(exc.stdout or b""),
                       stderr=(exc.stderr or b"").decode("utf-8", errors="replace"), **partial)
            return None
        except OSError as exc:
            self.error("adb_launch", phase=phase, serial=serial, command=command,
                       returncode=None, detail=str(exc))
            return None
        self.event("adb_read", phase=phase, serial=serial, command=command,
                   started_utc=started, returncode=result.returncode,
                   stdout_bytes=len(result.stdout), stdout_sha256=digest(result.stdout),
                   stderr=result.stderr.decode("utf-8", errors="replace"))
        if result.returncode:
            self.error("adb_read_failed", phase=phase, serial=serial, command=command,
                       returncode=result.returncode,
                       detail=result.stderr.decode("utf-8", errors="replace"))
        return result

    def shell(self, serial, script, arguments, phase, deadline):
        command = "sh -c " + shlex.quote(script) + " capture "
        command += " ".join(shlex.quote(str(value)) for value in arguments)
        return self.adb_call(["-s", serial, "exec-out", command], phase, serial, deadline)

    def enumerate(self, phase, deadline):
        result = self.adb_call(["devices", "-l"], phase, deadline=deadline)
        devices = {}
        if result is not None and result.returncode == 0:
            for line in result.stdout.decode("utf-8", errors="replace").splitlines():
                if not line.strip() or line.startswith(("List of devices", "*")):
                    continue
                fields = line.split()
                if len(fields) < 2:
                    self.error("adb_inventory_parse", phase=phase, detail=line)
                    continue
                serial, state = fields[:2]
                devices[serial] = {"state": state, "description": " ".join(fields[2:])}
                if state != "device":
                    self.error("adb_device_unavailable", phase=phase, serial=serial, state=state)
            if not devices:
                self.error("no_adb_devices", phase=phase)
        self.metadata["adb_enumerations"].append(self.event("adb_enumeration", phase=phase, devices=devices))
        return devices

    def inventory(self, serial, phase, deadline):
        result = self.shell(serial, INVENTORY_SH, [REMOTE_DIR], phase, deadline)
        if result is None:
            return None
        files = {}
        remote_status = None
        parse_ok = True
        for line in result.stdout.decode("utf-8", errors="replace").splitlines():
            try:
                if line.startswith("CAPTURE_INVENTORY_STATUS|"):
                    remote_status = int(line.split("|", 1)[1])
                    continue
                path, stat_text = line.split("|", 1)
                basename = Path(path).name
                if Path(path).parent.as_posix() != REMOTE_DIR or not CAMERA_FILE.fullmatch(basename):
                    continue  # Excludes temporary destinations and unrelated files.
                files[path] = remote_stat(stat_text)
            except ValueError as exc:
                parse_ok = False
                self.error("device_inventory_parse", phase=phase, serial=serial, detail=str(exc), line=line)
        expected_empty_baseline = (phase == "baseline" and result.returncode == 0
                                   and remote_status in (0, 3) and parse_ok and not files)
        complete = result.returncode == 0 and parse_ok and (remote_status == 0 or expected_empty_baseline)
        self.event("device_inventory", phase=phase, serial=serial, files=files,
                   returncode=result.returncode, remote_status=remote_status, complete=complete,
                   expected_empty_baseline=expected_empty_baseline,
                   source_directory_absent=remote_status == 3)
        self.inventory_complete[(serial, phase)] = complete
        if expected_empty_baseline:
            pass  # Firmware creates the directory/files only after the first test launches.
        elif remote_status == 3:
            self.error("device_debug_directory_missing", phase=phase, serial=serial,
                       directory=REMOTE_DIR, returncode=result.returncode, remote_status=remote_status)
        elif not complete:
            self.error("device_inventory_incomplete", phase=phase, serial=serial,
                       returncode=result.returncode, remote_status=remote_status,
                       detail=result.stderr.decode("utf-8", errors="replace"))
        elif not files:
            self.error("no_completed_device_json", phase=phase, serial=serial, directory=REMOTE_DIR)
        return files  # Preserve readable destinations even if another pathname vanished during inventory.

    def save_bytes(self, data, target):
        sha = digest(data)
        key = (str(target), sha)
        if key in self.saved:
            return self.saved[key], sha
        if target.exists():
            target = target.parent / "versions" / sha / target.name
        target.parent.mkdir(parents=True, exist_ok=True)
        with target.open("xb") as output:
            output.write(data)
        relative = target.relative_to(self.directory).as_posix()
        self.saved[key] = relative
        return relative, sha

    def read_device(self, serial, path, inventory_stat, phase, deadline):
        markers = ["CAPTURE_" + uuid.uuid4().hex for _ in range(3)]
        result = self.shell(serial, READ_SH, [path, *markers], phase, deadline)
        if result is None:
            return
        # Random framing separates exact bytes from before/after stat observations.
        # Stat and cat open the pathname independently: even equal stats are not an atomic claim.
        try:
            prefix = (markers[0] + "\n").encode()
            middle = (markers[1] + "\n").encode()
            suffix = ("\n" + markers[2] + "\n").encode()
            if not result.stdout.startswith(prefix):
                raise ValueError("missing read prefix")
            before_text, rest = result.stdout[len(prefix):].split(middle, 1)
            data, after_text = rest.rsplit(suffix, 1)
            after_lines = after_text.decode("ascii").splitlines()
            statuses = list(map(int, after_lines[-1].split("|")))
            if len(statuses) != 3:
                raise ValueError("missing stat/read statuses")
            before = remote_stat(before_text.decode("ascii")) if statuses[0] == 0 else None
            after = remote_stat("\n".join(after_lines[:-1])) if statuses[2] == 0 else None
        except (ValueError, IndexError, UnicodeError) as exc:
            # Keep even malformed protocol output as explicit, excluded evidence.
            relative, sha = self.save_bytes(result.stdout, self.directory / "device" / serial_component(serial)
                                            / "read-errors" / (Path(path).name + ".read-output"))
            self.error("device_read_framing", phase=phase, serial=serial, source=path,
                       detail=str(exc), destination=relative, sha256=sha, returncode=result.returncode)
            return
        relative, sha = self.save_bytes(data, self.directory / "device" / serial_component(serial) / Path(path).name)
        observed = ("new_path" if (serial, path) not in self.baseline and serial in self.metadata["baseline_serials"]
                    else "changed_stat" if self.baseline.get((serial, path)) != before
                    and (serial, path) in self.baseline else "unchanged_stat_or_unknown_baseline")
        record = self.event("device_json", phase=phase, serial=serial, source=path,
                            destination=relative, bytes=len(data), sha256=sha,
                            inventory_stat=inventory_stat, stat_before=before, stat_after=after,
                            stat_read_statuses=statuses, returncode=result.returncode,
                            stable_observation=before == after and before is not None
                            and before["size"] == len(data) and statuses == [0, 0, 0],
                            atomic_snapshot=False, baseline_observation=observed)
        self.metadata["deviceJSONcopies"].append(record)
        if not record["stable_observation"]:
            self.error("device_read_unstable_or_partial", phase=phase, serial=serial, source=path,
                       destination=relative, returncode=result.returncode, stat_read_statuses=statuses)
        try:
            document = json_object(data)
            record["identity"] = {field: document.get(field) for field in DEVICE_FIELDS}
            camera, slot, role = CAMERA_FILE.fullmatch(Path(path).name).groups()
            record["filename_identity"] = {"camera_id": int(camera), "pid_slot": int(slot), "role": role}
            record["match"] = {"classification": "unknown", "evidence": [], "host_streams": []}
            if document.get("schema") != "depthai-camera-debug-v1":
                self.error("device_schema_unknown", phase=phase, serial=serial, source=path,
                           schema=document.get("schema"))
            pid = document.get("pid")
            if document.get("camera_id") != int(camera) or not isinstance(pid, int) or pid % 16 != int(slot):
                self.error("device_filename_identity_mismatch", phase=phase, serial=serial, source=path)
            if not isinstance(pid, int) or pid <= 0:
                raise ValueError("pid must be a positive integer")
            for field in ("process_session", "session"):
                if not isinstance(document.get(field), str) or not document[field]:
                    raise ValueError(field + " must be a nonempty string")
            if not isinstance(document.get("events", []), list):
                raise ValueError("events must be an array")
            frames = set()
            for event in document.get("events", []):
                if isinstance(event, dict) and "sequence_num" in event and "device_timestamp_ns" in event:
                    key = (event["sequence_num"], event["device_timestamp_ns"], event.get("system_timestamp_ns"))
                    if all(isinstance(value, int) for value in key):
                        frames.add(key)
            self.device_frames[relative] = frames
        except (ValueError, TypeError, UnicodeError) as exc:
            record["parse_error"] = str(exc)
            self.error("device_json_parse", phase=phase, serial=serial, source=path,
                       destination=relative, detail=str(exc))
        self.event("device_json_details", destination=relative, identity=record.get("identity"),
                   filename_identity=record.get("filename_identity"), parse_error=record.get("parse_error"))
        if phase == "poll":
            self.poll_bytes += len(data)

    def baseline_capture(self):
        deadline = time.monotonic() + self.args.collection_timeout
        devices = self.enumerate("start", deadline)
        for serial, info in devices.items():
            if info["state"] != "device":
                continue
            files = self.inventory(serial, "baseline", deadline)
            if files is not None:
                if self.inventory_complete[(serial, "baseline")]:
                    self.metadata["baseline_serials"].append(serial)
                self.baseline.update({(serial, path): stat for path, stat in files.items()})
                self.last_inventory.update({(serial, path): stat for path, stat in files.items()})

    def collect_devices(self, phase):
        budget = self.args.poll_budget if phase == "poll" else self.args.collection_timeout
        deadline = time.monotonic() + budget
        devices = self.enumerate(phase, deadline)
        if phase == "finish":
            initial = self.metadata["adb_enumerations"][0]["devices"]
            for serial in initial:
                if serial not in devices:
                    self.error("adb_device_disappeared", phase=phase, serial=serial)
        online_serials = [serial for serial, info in devices.items() if info["state"] == "device"]
        pending = []
        # Inventory every board before copying older slots from any one board.
        for device_index, serial in enumerate(online_serials):
            if phase == "poll" and self.stop.is_set():
                return
            if time.monotonic() >= deadline:
                self.error("collection_cycle_incomplete", phase=phase, stage="inventory", serial=serial,
                           remaining_inventory_serials=online_serials[device_index:],
                           remaining_paths=[{"serial": board, "source": path} for board, path, _ in pending])
                return
            files = self.inventory(serial, phase, deadline)
            if files is None:
                continue
            pending.extend((serial, path, stat) for path, stat in files.items()
                           if phase != "poll" or self.last_inventory.get((serial, path)) != stat)
        # Global priority: new/changed destinations on ALL boards precede unchanged older slots.
        comparison = self.last_inventory if phase == "poll" else self.baseline
        pending.sort(key=lambda item: comparison.get((item[0], item[1])) == item[2])
        for file_index, (serial, path, stat) in enumerate(pending):
            if phase == "poll" and self.stop.is_set():
                return
            if time.monotonic() >= deadline:
                self.error("collection_cycle_incomplete", phase=phase, stage="read", serial=serial, next_source=path,
                           remaining_paths=[{"serial": board, "source": source}
                                            for board, source, _ in pending[file_index:]])
                return
            if phase == "poll":
                if (self.poll_copies >= self.args.poll_max_copies
                        or self.poll_bytes + stat["size"] > self.args.poll_max_bytes):
                    if not self.poll_limit_reported:
                        self.error("poll_limit_reached", phase=phase, serial=serial, next_source=path,
                                   copies=self.poll_copies, bytes=self.poll_bytes)
                        self.poll_limit_reported = True
                    return
                self.poll_copies += 1
            try:
                self.read_device(serial, path, stat, phase, deadline)
            except OSError as exc:
                self.error("device_snapshot_local_write", phase=phase, serial=serial, source=path, detail=str(exc))
            self.last_inventory[(serial, path)] = stat

    def start_polling(self):
        if not self.args.poll_interval or self.adb is None:
            return

        def poll():
            try:
                while not self.stop.wait(self.args.poll_interval):
                    self.collect_devices("poll")
                    if self.poll_limit_reported:
                        break
            except Exception as exc:
                self.error("poll_worker", detail=str(exc))

        self.worker = threading.Thread(target=poll, name="camera-json-capture")
        self.worker.start()

    def stop_polling(self):
        self.stop.set()
        if self.worker is not None and self.worker.ident is not None:
            # Each subprocess has a timeout; the worker checks stop between file reads.
            self.worker.join()

    def capture_hosts(self):
        references = []
        with (self.directory / "attempt.log").open(encoding="utf-8", errors="replace") as log:
            for line in log:
                marker = "Sync debug artifact:"
                role = re.search(r"\b(\S+) is (master|slave)\s*$", line)
                if role:
                    self.metadata["host_device_roles"][role.group(1)] = role.group(2)
                if marker not in line:
                    continue
                text = line.split(marker, 1)[1].strip()
                if text.startswith('"'):
                    quoted = re.fullmatch(r'"((?:\\.|[^"\\])*)"', text)
                    if quoted is None:
                        self.error("host_artifact_path_parse", printed_path=text)
                        continue
                    text = re.sub(r'\\(["\\])', r'\1', quoted.group(1))
                elif len(text) >= 2 and text[0] == text[-1] == "'":
                    text = text[1:-1]
                references.append((text, "stdout_reference"))
        live = self.directory / "host-debug-live"
        if live.exists():
            references.extend((str(path), "attempt_owned_debug_directory") for path in sorted(live.glob("*.json")))
        if self.args.host_debug_dir:
            root = self.args.host_debug_dir.resolve()
            for path in sorted(root.glob("*.json")):
                try:
                    stat = local_stat(path)
                    previous = self.metadata["host_debug_baseline"].get(str(path))
                    if previous != stat:
                        references.append((str(path), "custom_directory_new_or_changed"))
                except OSError as exc:
                    self.error("host_directory_read", source=str(path), detail=str(exc))
        host_documents = []
        seen = set()
        for text, provenance in references:
            path = Path(text)
            if not path.is_absolute():
                path = Path(self.metadata["cwd"]) / path
            try:
                path = path.resolve()
                if str(path) in seen:
                    continue
                seen.add(str(path))
                if not path.is_file():
                    raise OSError("referenced artifact is not an existing regular file")
                before = local_stat(path)
                data = path.read_bytes()
                after = local_stat(path)
                relative, sha = self.save_bytes(data, self.directory / "host-debug" / path.name)
                record = self.event("host_json", source=str(path), printed_path=text,
                                    provenance=provenance, destination=relative, bytes=len(data), sha256=sha,
                                    stat_before=before, stat_after=after, atomic_snapshot=False,
                                    stable_observation=before == after and before["size"] == len(data))
                self.metadata["hostArtifacts"].append(record)
                if not record["stable_observation"]:
                    self.error("host_read_unstable", source=str(path), destination=relative)
                document = json_object(data)
                record["identity"] = {field: document.get(field) for field in (
                    "schema_version", "success", "reason", "started_system_ns", "started_steady_ns",
                    "frozen_steady_ns", "configuration", "diagnostic_extensions")}
                record["streams"] = [{"id": stream.get("id"), "name": stream.get("name"),
                                      "device_id": stream.get("device_id")}
                                     for stream in document.get("streams", []) if isinstance(stream, dict)]
                host_documents.append((relative, document))
            except (OSError, ValueError, TypeError, UnicodeError) as exc:
                self.error("host_artifact_read_or_parse", source=str(path), provenance=provenance, detail=str(exc))
        if not self.metadata["hostArtifacts"]:
            self.error("no_host_artifacts", host_debug_enabled=self.metadata["host_debug_enabled"])
        return host_documents

    def classify(self, hosts):
        frame_index = {}
        for artifact, document in hosts:
            for stream in document.get("streams", []):
                if not isinstance(stream, dict):
                    continue
                identity = {"artifact": artifact, "device_id": stream.get("device_id"),
                            "stream_id": stream.get("id"), "name": stream.get("name")}
                identity["fsync_role"] = self.metadata["host_device_roles"].get(str(stream.get("device_id")))
                arrival = stream.get("arrival", {})
                if not isinstance(arrival, dict) or not isinstance(arrival.get("events", []), list):
                    self.error("host_history_shape", artifact=artifact, stream_id=stream.get("id"))
                    continue
                for event in arrival.get("events", []):
                    if not isinstance(event, dict):
                        continue
                    key = (event.get("sequence"), event.get("device_ns"), event.get("system_ns"))
                    if all(isinstance(value, int) for value in key):
                        frame_index.setdefault(key, []).append(identity)
        linked_sessions = set()
        for record in self.metadata["deviceJSONcopies"]:
            identity = record.get("identity")
            if not identity or record.get("parse_error"):
                continue
            match = record["match"]
            matches = {}
            for frame in self.device_frames.get(record["destination"], set()):
                for stream in frame_index.get(frame, []):
                    key = json.dumps(stream, sort_keys=True)
                    matches.setdefault(key, {**stream, "exact_frame_matches": 0})["exact_frame_matches"] += 1
            match["host_streams"] = list(matches.values())
            started = identity.get("system_start_ns")
            if isinstance(started, int):
                if self.metadata["launch_system_ns"] <= started <= self.metadata["test_finished_system_ns"]:
                    match["evidence"].append("system_start_within_host_attempt_clock_window")
                    match["classification"] = "current_attempt_time_candidate"
                elif started < self.metadata["launch_system_ns"]:
                    match["evidence"].append("system_start_before_host_attempt_clock_window")
                    match["classification"] = "existing_time_candidate"
            if record["baseline_observation"] != "unchanged_stat_or_unknown_baseline":
                match["evidence"].append(record["baseline_observation"])
                if match["classification"] == "unknown":
                    match["classification"] = "changed_or_new_unknown_session"
            session_key = (record["serial"], identity.get("pid"), identity.get("process_session"))
            if matches:
                match["classification"] = "host_frame_correlated"
                match["evidence"].append("exact_sequence_device_and_system_timestamp_join")
                if (isinstance(identity.get("pid"), int) and isinstance(identity.get("process_session"), str)
                        and identity["process_session"]):
                    linked_sessions.add(session_key)
        for record in self.metadata["deviceJSONcopies"]:
            identity = record.get("identity", {})
            if record.get("parse_error"):
                continue
            key = (record["serial"], identity.get("pid"), identity.get("process_session"))
            if key in linked_sessions and record["match"]["classification"] != "host_frame_correlated":
                record["match"]["classification"] = "host_correlated_process_session"
                record["match"]["evidence"].append("same_serial_pid_process_session_as_host_frame_join")
            if "match" in record:
                self.event("device_attribution", destination=record["destination"], serial=record["serial"],
                           identity=identity, match=record["match"])
        coverage = {}
        for enumeration in self.metadata["adb_enumerations"]:
            for serial in enumeration["devices"]:
                coverage.setdefault(serial, {"copied_snapshots": 0, "classification_counts": {}})
        for record in self.metadata["deviceJSONcopies"]:
            counts = coverage[record["serial"]]
            counts["copied_snapshots"] += 1
            classification = "parse_error" if record.get("parse_error") else record.get("match", {}).get("classification", "unknown")
            counts["classification_counts"][classification] = counts["classification_counts"].get(classification, 0) + 1
        self.metadata["device_session_coverage"] = coverage


def run_test(args, command, environment, directory, metadata, interrupted, capture):
    metadata["launch_utc"] = utc_now()
    metadata["launch_system_ns"] = time.time_ns()
    process = None
    with (directory / "attempt.log").open("xb") as log:
        try:
            process = subprocess.Popen(command, cwd=args.cwd, env=environment, stdout=subprocess.PIPE,
                                       stderr=subprocess.STDOUT, start_new_session=True, bufsize=0)
        except (OSError, ValueError) as exc:
            metadata["test_result"] = "launch_error"
            metadata["test_returncode"] = None
            metadata["launch_error"] = str(exc)
            log.write(("Runner process launch failed: " + str(exc) + "\n").encode())
        if process is not None:
            selector = selectors.DefaultSelector()
            started = time.monotonic()
            escalation = None
            finished = None
            console_available = True
            unreaped = False
            try:
                metadata["host_process_pid"] = process.pid
                metadata["test_result"] = "running"
                write_json(directory / "metadata.json", metadata)
                capture.start_polling()
                selector.register(process.stdout, selectors.EVENT_READ)
                while True:
                    now = time.monotonic()
                    returncode = process.poll()
                    if returncode is None and escalation is None:
                        if interrupted.is_set() or args.test_timeout and now - started >= args.test_timeout:
                            metadata["termination_reason"] = "interrupt" if interrupted.is_set() else "test_timeout"
                            escalation = (now + args.interrupt_grace, signal.SIGTERM)
                            forward_signal(process, signal.SIGINT, metadata)
                    elif returncode is None and escalation and now >= escalation[0]:
                        if escalation[1] is None:
                            unreaped = True
                            capture.stop_polling()
                            capture.error("test_process_unreaped", pid=process.pid,
                                          detail="test did not exit within 5 seconds of SIGKILL; stopping the batch")
                            break
                        forward_signal(process, escalation[1], metadata)
                        escalation = (now + 5, signal.SIGKILL if escalation[1] == signal.SIGTERM else None)
                    if returncode is not None and finished is None:
                        finished = now
                        metadata["test_finished_utc"] = utc_now()
                        metadata["test_finished_system_ns"] = time.time_ns()
                        capture.stop.set()
                    for key, _ in selector.select(0.1):
                        chunk = os.read(key.fileobj.fileno(), 65536)
                        if not chunk:
                            selector.unregister(key.fileobj)
                        else:
                            log.write(chunk)
                            log.flush()
                            if console_available:
                                try:
                                    if hasattr(sys.stdout, "buffer"):
                                        sys.stdout.buffer.write(chunk)
                                        sys.stdout.buffer.flush()
                                    else:
                                        sys.stdout.write(chunk.decode("utf-8", errors="replace"))
                                        sys.stdout.flush()
                                except (BrokenPipeError, OSError):
                                    console_available = False
                    if finished is not None and not selector.get_map():
                        break
                    if finished is not None and now - finished >= 2:
                        capture.stop_polling()
                        capture.error("test_output_pipe_still_open", detail="closed after test exit; a descendant may retain stdout")
                        break
                metadata["test_returncode"] = None if unreaped else process.wait()
                metadata["test_result"] = ("termination_unreaped" if unreaped
                                           else "interrupted" if metadata.get("termination_reason") == "interrupt"
                                           else "timeout" if metadata.get("termination_reason") == "test_timeout"
                                           else "pass" if process.returncode == 0 else "fail")
            except Exception as exc:
                capture.stop_polling()
                was_running = process.poll() is None
                metadata["test_returncode"] = stop_owned_test(process, metadata)
                metadata["test_result"] = ("runner_error" if was_running else "pass" if process.returncode == 0 else "fail")
                metadata["runner_abort"] = True
                capture.error("test_process_supervision", pid=process.pid, detail=str(exc))
            finally:
                selector.close()
                process.stdout.close()
                capture.stop_polling()
        log.flush()
        os.fsync(log.fileno())
    metadata.setdefault("test_finished_utc", utc_now())
    metadata.setdefault("test_finished_system_ns", time.time_ns())


def forward_signal(process, signum, metadata):
    # The test has its own session/group. Never signal the runner's group or ADB devices.
    try:
        os.killpg(process.pid, signum)
        metadata.setdefault("forwarded_signals", []).append({"signal": signum, "utc": utc_now()})
    except ProcessLookupError:
        pass


def stop_owned_test(process, metadata):
    for signum in (signal.SIGTERM, signal.SIGKILL):
        if process.poll() is not None:
            return process.returncode
        forward_signal(process, signum, metadata)
        try:
            return process.wait(timeout=5)
        except subprocess.TimeoutExpired:
            pass
    return None


def archive_batch(batch):
    manifest = batch / "sha256-manifest.jsonl"
    paths = sorted(path for path in batch.rglob("*") if path.is_file() and not path.is_symlink())
    with manifest.open("x", encoding="utf-8") as output:
        for path in paths:
            with path.open("rb") as source:
                sha = hashlib.sha256()
                for chunk in iter(lambda: source.read(1024 * 1024), b""):
                    sha.update(chunk)
            output.write(json.dumps({"sha256": sha.hexdigest(), "path": path.relative_to(batch).as_posix()}) + "\n")
    archive = batch.with_name(batch.name + ".tar.xz")
    partial = archive.with_suffix(archive.suffix + ".partial")
    with partial.open("xb") as output:
        with tarfile.open(fileobj=output, mode="w:xz") as tar:
            tar.add(batch, arcname=batch.name, recursive=False)
            for path in sorted(batch.rglob("*")):
                if not path.is_symlink():
                    tar.add(path, arcname=batch.name + "/" + path.relative_to(batch).as_posix(), recursive=False)
    # Hard-link publication fails rather than replacing any existing archive.
    os.link(partial, archive)
    partial.unlink()
    return archive


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    parser.add_argument("--consecutive-successes", type=positive_int, default=50)
    parser.add_argument("--max-attempts", type=positive_int, default=200)
    parser.add_argument("--output-dir", type=Path, default=Path("test-logs"), help="parent of a new unique batch directory")
    parser.add_argument("--cwd", type=Path, default=Path.cwd(), help="test working directory; relative artifact paths resolve here")
    parser.add_argument("--enable-host-debug", action="store_true", help="set DEPTHAI_SYNC_DEBUG=1 and an attempt-owned debug directory")
    parser.add_argument("--host-debug-dir", type=Path, help="also snapshot new/changed JSONs in this custom local directory")
    parser.add_argument("--adb", default="adb", help="ADB client executable, found using PATH")
    parser.add_argument("--adb-timeout", type=positive_float, default=5, help="seconds per ADB call")
    parser.add_argument("--collection-timeout", type=positive_float, default=180, help="seconds for each baseline/final device collection")
    parser.add_argument("--poll-interval", type=poll_interval, default=0, help="seconds between optional in-test device collection cycles")
    parser.add_argument("--poll-budget", type=positive_float, default=20, help="maximum seconds per polling cycle")
    parser.add_argument("--poll-max-copies", type=positive_int, default=512, help="maximum in-test file reads per attempt")
    parser.add_argument("--poll-max-bytes", type=positive_int, default=256 * 1024 * 1024, help="in-test byte budget per attempt; final collection is separate")
    parser.add_argument("--test-timeout", type=positive_float, help="optional test duration limit in seconds")
    parser.add_argument("--interrupt-grace", type=positive_float, default=10, help="seconds after forwarding SIGINT before SIGTERM, then SIGKILL after 5 seconds")
    parser.add_argument("--archive", action="store_true", help="create a SHA-256 manifest and a sibling immutable .tar.xz at batch completion")
    parser.add_argument("command", nargs=argparse.REMAINDER, help="-- executable arguments (forwarded without a shell)")
    args = parser.parse_args()
    if args.max_attempts < args.consecutive_successes:
        parser.error("--max-attempts must be at least --consecutive-successes")
    args.cwd = args.cwd.resolve()
    if not args.cwd.is_dir():
        parser.error("--cwd must be an existing directory")
    command = args.command[1:] if args.command[:1] == ["--"] else args.command
    if not command:
        command = [str(Path(__file__).resolve().parents[1] / "build/tests/multi_device_fsync_test"), "[fps-60]"]
    return args, command


def main():
    args, command = parse_args()
    interrupted = threading.Event()
    for signum in (signal.SIGINT, signal.SIGTERM):
        signal.signal(signum, lambda _signum, _frame: interrupted.set())
    adb = shutil.which(args.adb)
    if adb is None:
        print("Capture error: ADB client not found: " + args.adb
              + ". Tests will run, but device capture is incomplete; use --adb /path/to/adb.", file=sys.stderr)
    print("Firmware recorder enablement is not verified by host environment. The firmware must already use "
          "DEPTHAI_CAMERA_DEBUG=1 and " + REMOTE_DIR + ".", file=sys.stderr)
    args.output_dir.mkdir(parents=True, exist_ok=True)
    batch = args.output_dir.resolve() / ("sync-attempts-" + utc_name() + "-" + uuid.uuid4().hex[:8])
    batch.mkdir()
    print("Batch: " + str(batch), flush=True)
    batch_metadata = {"schema_version": 1, "started_utc": utc_now(), "command": command,
                      "cwd": str(args.cwd), "consecutive_successes_target": args.consecutive_successes,
                      "max_attempts": args.max_attempts, "adb": adb, "remote_debug_directory": REMOTE_DIR,
                      "firmware_debug_enablement": "not_verified", "options": {
                          key: str(value) if isinstance(value, Path) else value
                          for key, value in vars(args).items() if key != "command"}, "attempts": []}
    write_json(batch / "metadata.json", batch_metadata)
    with (batch / "metadata.txt").open("x", encoding="utf-8") as output:
        output.write("Started UTC: " + batch_metadata["started_utc"] + "\nCommand: " + shlex.join(command)
                     + "\nCWD: " + str(args.cwd) + "\nConsecutive successes target: " + str(args.consecutive_successes)
                     + "\nMax attempts: " + str(args.max_attempts) + "\nNo environment dump recorded.\n")
    streak = 0
    capture_incomplete = False
    launch_failed = False
    with (batch / "summary.tsv").open("x", encoding="utf-8", newline="") as summary:
        writer = csv.writer(summary, delimiter="\t", lineterminator="\n")
        writer.writerow(["attempt", "started_utc", "test_result", "test_exit", "consecutive_successes",
                         "collection_status", "host_jsons", "device_json_copies", "collection_errors", "attempt_directory"])
        summary.flush()
        for index in range(1, args.max_attempts + 1):
            if interrupted.is_set():
                break
            started = utc_now()
            directory = batch / ("attempt_" + str(index).zfill(4) + "_" + utc_name())
            directory.mkdir()
            metadata = {"schema_version": 1, "attempt": index, "started_utc": started,
                        "command": command, "cwd": str(args.cwd), "test_returncode": None,
                        "test_result": "not_started", "hostArtifacts": [], "deviceJSONcopies": [],
                        "collection_errors": [], "adb_enumerations": [], "baseline_serials": [],
                        "host_debug_baseline": {}, "host_device_roles": {}, "firmware_debug_enablement": "not_verified"}
            environment = os.environ.copy()
            enabled = args.enable_host_debug or environment.get("DEPTHAI_SYNC_DEBUG") == "1"
            metadata["host_debug_enabled"] = enabled
            if enabled:
                live = directory / "host-debug-live"
                live.mkdir()
                environment["DEPTHAI_SYNC_DEBUG"] = "1"
                environment["DEPTHAI_SYNC_DEBUG_DIR"] = str(live)
                metadata["host_debug_directory"] = str(live)
            capture = Capture(args, directory, metadata, adb)
            try:
                if adb is None:
                    capture.error("adb_missing", executable=args.adb)
                if args.host_debug_dir:
                    if not args.host_debug_dir.is_dir():
                        capture.error("host_debug_directory_missing", source=str(args.host_debug_dir))
                    for path in sorted(args.host_debug_dir.resolve().glob("*.json")):
                        try:
                            metadata["host_debug_baseline"][str(path)] = local_stat(path)
                        except OSError as exc:
                            capture.error("host_baseline_read", source=str(path), detail=str(exc))
                write_json(directory / "metadata.json", metadata)
                capture.baseline_capture()
                if interrupted.is_set():
                    metadata["test_result"] = "interrupted_before_launch"
                    metadata["launch_system_ns"] = time.time_ns()
                    metadata["test_finished_system_ns"] = time.time_ns()
                    (directory / "attempt.log").touch(exist_ok=False)
                else:
                    run_test(args, command, environment, directory, metadata, interrupted, capture)
                write_json(directory / "metadata.json", metadata)
                # This barrier completes for both pass and fail BEFORE the next test is launched.
                capture.collect_devices("finish")
                try:
                    hosts = capture.capture_hosts()
                    capture.classify(hosts)
                except (OSError, ValueError, TypeError, AttributeError) as exc:
                    capture.error("host_collection_or_attribution", detail=str(exc))
            finally:
                capture.stop_polling()
                capture.manifest.close()
            streak = streak + 1 if metadata["test_result"] == "pass" else 0
            launch_failed = metadata["test_result"] in ("launch_error", "termination_unreaped") or metadata.get("runner_abort", False)
            metadata["finished_utc"] = utc_now()
            metadata["consecutive_successes"] = streak
            metadata["collection_status"] = "incomplete" if metadata["collection_errors"] else "completed_reads_session_coverage_unverified"
            metadata["coverage_note"] = "Completed files observed, not proof of all firmware sessions or failure-window histories; see match evidence."
            metadata["counts"] = {"host_jsons": len(metadata["hostArtifacts"]),
                                  "device_json_copies": len(metadata["deviceJSONcopies"]),
                                  "unique_device_snapshots": len({record["destination"] for record in metadata["deviceJSONcopies"]}),
                                  "collection_errors": len(metadata["collection_errors"])}
            capture_incomplete |= bool(metadata["collection_errors"])
            write_json(directory / "metadata.json", metadata)
            writer.writerow([index, started, metadata["test_result"], metadata["test_returncode"], streak,
                             metadata["collection_status"], metadata["counts"]["host_jsons"],
                             metadata["counts"]["device_json_copies"], metadata["counts"]["collection_errors"], directory.name])
            summary.flush()
            os.fsync(summary.fileno())
            batch_metadata["attempts"].append({"directory": directory.name, "test_result": metadata["test_result"],
                                               "test_returncode": metadata["test_returncode"], "collection_status": metadata["collection_status"]})
            write_json(batch / "metadata.json", batch_metadata)
            print("Attempt {}: {}, exit={}, streak={}, capture={}".format(
                index, metadata["test_result"], metadata["test_returncode"], streak, metadata["collection_status"]), flush=True)
            if launch_failed or streak >= args.consecutive_successes or interrupted.is_set():
                break
    exit_code = (130 if interrupted.is_set() else 3 if launch_failed else 1
                 if streak < args.consecutive_successes else 2 if capture_incomplete else 0)
    batch_metadata.update(finished_utc=utc_now(), consecutive_successes=streak,
                          target_reached=streak >= args.consecutive_successes,
                          capture_incomplete=capture_incomplete, runner_exit_code=exit_code)
    write_json(batch / "metadata.json", batch_metadata)
    if args.archive:
        try:
            print("Archive: " + str(archive_batch(batch)), flush=True)
        except (OSError, tarfile.TarError) as exc:
            print("Capture error: archive failed: " + str(exc), file=sys.stderr)
            if exit_code == 0:
                exit_code = 2
            batch_metadata.update(archive_error=str(exc), capture_incomplete=True, runner_exit_code=exit_code)
            write_json(batch / "metadata.json", batch_metadata)
    print("Summary: " + str(batch / "summary.tsv"), flush=True)
    return exit_code


if __name__ == "__main__":
    sys.exit(main())
