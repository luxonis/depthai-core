"""Behavioral tests for the debugging helpers; no SDK import, build or device."""

import hashlib
import gzip
import io
import json
import os
from pathlib import Path
import signal
import struct
import subprocess
import sys
import tarfile
import tempfile
import time
import unittest

SCRIPTS = Path(__file__).resolve().parents[3] / ".agents/skills/depthai-debugger/scripts"


class HelpersTest(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory(prefix="depthai skill tests ")
        self.addCleanup(self.temporary.cleanup)
        self.root = Path(self.temporary.name)

    def run_script(self, script, *arguments, environment=None):
        env = os.environ.copy()
        env["PYTHONDONTWRITEBYTECODE"] = "1"
        if environment:
            env.update(environment)
        return subprocess.run(
            [sys.executable, str(SCRIPTS / script), *map(str, arguments)],
            cwd=self.root, env=env, capture_output=True, text=True, timeout=20,
        )

    def archive(self, filename, members, mode="w", format=tarfile.PAX_FORMAT):
        path = self.root / filename
        with tarfile.open(path, mode, format=format) as archive:
            for name, data in members:
                info = name if isinstance(name, tarfile.TarInfo) else tarfile.TarInfo(name)
                info.size = len(data)
                archive.addfile(info, io.BytesIO(data))
        return path

    def inspect(self, artifact, *arguments, directory="inspection"):
        output = self.root / directory
        result = self.run_script("inspect_crashdump.py", artifact, "--output-dir", output, *arguments)
        report_path = output / "inspection.json"
        report = json.loads(report_path.read_text()) if report_path.exists() else None
        return result, report, output

    def test_context_does_not_import_sdk_or_collect_secrets(self):
        marker = self.root / "imported"
        module = self.root / "depthai.py"
        module.write_text("from pathlib import Path\nPath(" + repr(str(marker)) + ").touch()\n")
        output = self.root / "context.json"
        result = self.run_script(
            "collect_context.py", "--repo", self.root, "--output", output,
            environment={"DEPTHAI_LEVEL": "debug", "DEPTHAI_TELEMETRY_API_KEY": "private-test-value"},
        )
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertFalse(marker.exists())
        report = json.loads(output.read_text())
        self.assertEqual(report["python"]["depthai_module_candidate"], str(module))
        self.assertEqual(report["environment"]["DEPTHAI_LEVEL"], "debug")
        self.assertNotIn("private-test-value", output.read_text())
        self.assertNotEqual(report["repositories"][0]["commit"].get("returncode"), 0)

    def test_context_artifact_and_cache(self):
        artifact = self.root / "firmware with spaces"
        artifact.write_bytes(b"preserved firmware")
        (self.root / "CMakeCache.txt").write_text(
            "CMAKE_BUILD_TYPE:STRING=RelWithDebInfo\n"
            "DEPTHAI_SANITIZE:BOOL=ON\n"
            "DEPTHAI_TEST_API_KEY:STRING=private-test-value\n"
        )
        output = self.root / "context.json"
        result = self.run_script(
            "collect_context.py", "--repo", self.root, "--output", output,
            "--artifact", artifact, "--build-dir", self.root,
        )
        self.assertEqual(result.returncode, 0, result.stderr)
        report = json.loads(output.read_text())
        self.assertEqual(report["artifacts"][0]["sha256"], hashlib.sha256(artifact.read_bytes()).hexdigest())
        self.assertEqual(report["cmake_cache"]["values"]["CMAKE_BUILD_TYPE"], "RelWithDebInfo")
        self.assertNotIn("private-test-value", output.read_text())

    def test_context_never_overwrites(self):
        output = self.root / "context.json"
        output.write_text("preserved")
        result = self.run_script("collect_context.py", "--output", output)
        self.assertEqual(result.returncode, 2)
        self.assertEqual(output.read_text(), "preserved")

    def test_capture_preserves_arguments_logs_and_exit(self):
        output = self.root / "run"
        argument = "$(touch must-not-exist); literal spaces"
        result = self.run_script(
            "capture_run.py", "--output-dir", output, "--", sys.executable, "-c",
            "import sys; print(sys.argv[1]); print('error detail', file=sys.stderr); sys.exit(7)",
            argument,
        )
        self.assertEqual(result.returncode, 7, result.stderr)
        log = (output / "host.log").read_text()
        self.assertIn(argument, log)
        self.assertIn("error detail", log)
        self.assertFalse((self.root / "must-not-exist").exists())
        manifest = json.loads((output / "manifest.json").read_text())
        self.assertEqual(manifest["returncode"], 7)
        self.assertEqual(manifest["status"], "exited")
        self.assertEqual(manifest["cwd"], str(self.root))

    def test_capture_launch_failure_is_recorded(self):
        output = self.root / "run"
        result = self.run_script(
            "capture_run.py", "--output-dir", output, "--", self.root / "absent-executable",
        )
        self.assertEqual(result.returncode, 127)
        manifest = json.loads((output / "manifest.json").read_text())
        self.assertEqual(manifest["status"], "launch_or_capture_failed")
        self.assertIsNone(manifest["returncode"])

    def test_capture_never_overwrites(self):
        output = self.root / "run"
        output.mkdir()
        (output / "host.log").write_text("preserved")
        result = self.run_script("capture_run.py", "--output-dir", output, "--", sys.executable, "-c", "pass")
        self.assertEqual(result.returncode, 2)
        self.assertEqual((output / "host.log").read_text(), "preserved")

    @unittest.skipUnless(os.name == "posix", "POSIX signal behavior")
    def test_capture_timeout_is_not_a_successful_run(self):
        output = self.root / "run"
        result = self.run_script(
            "capture_run.py", "--output-dir", output, "--timeout", "0.6", "--grace", "1",
            "--", sys.executable, "-c",
            "import signal, sys, time; signal.signal(signal.SIGINT, lambda *_: sys.exit(0)); time.sleep(30)",
        )
        self.assertEqual(result.returncode, 124, result.stderr)
        manifest = json.loads((output / "manifest.json").read_text())
        self.assertEqual(manifest["status"], "timed_out")
        self.assertEqual(manifest["returncode"], 0)
        self.assertEqual(manifest["termination_actions"], ["interrupt"])

    @unittest.skipUnless(os.name == "posix", "POSIX signal behavior")
    def test_capture_escalates_if_interruption_is_ignored(self):
        output = self.root / "run"
        result = self.run_script(
            "capture_run.py", "--output-dir", output, "--timeout", "0.6", "--grace", "0.1",
            "--", sys.executable, "-c",
            "import signal, time; signal.signal(signal.SIGINT, signal.SIG_IGN); time.sleep(30)",
        )
        self.assertEqual(result.returncode, 124, result.stderr)
        manifest = json.loads((output / "manifest.json").read_text())
        self.assertEqual(manifest["returncode"], -signal.SIGTERM)
        self.assertEqual(manifest["termination_actions"], ["interrupt", "terminate"])

    @unittest.skipUnless(os.name == "posix", "POSIX signal behavior")
    def test_capture_repeated_interrupt_during_cleanup(self):
        for initial in ("interrupt", "timeout"):
            with self.subTest(initial=initial):
                pid_file = self.root / f"{initial}.pid"
                interrupted = self.root / f"{initial}.interrupted"
                child = (
                    "import os, signal, time\n"
                    "from pathlib import Path\n"
                    f"signal.signal(signal.SIGINT, lambda *_: Path({str(interrupted)!r}).touch())\n"
                    f"Path({str(pid_file)!r}).write_text(str(os.getpid()))\n"
                    "time.sleep(30)\n"
                )
                output = self.root / initial
                timeout = ["--timeout", "1"] if initial == "timeout" else []
                capture = subprocess.Popen(
                    [sys.executable, str(SCRIPTS / "capture_run.py"),
                     "--output-dir", str(output), "--grace", "1", *timeout,
                     "--", sys.executable, "-c", child],
                    stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True,
                )
                try:
                    for ready in (pid_file, interrupted):
                        deadline = time.monotonic() + 5
                        while not ready.exists() and capture.poll() is None and time.monotonic() < deadline:
                            time.sleep(0.01)
                        self.assertTrue(ready.exists(), f"did not reach {ready.name}")
                        if ready == interrupted or initial == "interrupt":
                            capture.send_signal(signal.SIGINT)
                    _, stderr = capture.communicate(timeout=5)
                    self.assertEqual(capture.returncode, 124 if timeout else 130, stderr)
                    manifest = json.loads((output / "manifest.json").read_text())
                    self.assertFalse(manifest["child_still_running"])
                    self.assertIn("terminate", manifest["termination_actions"])
                    with self.assertRaises(ProcessLookupError):
                        os.kill(int(pid_file.read_text()), 0)
                finally:
                    if pid_file.exists():
                        try:
                            os.kill(int(pid_file.read_text()), signal.SIGKILL)
                        except ProcessLookupError:
                            pass
                    if capture.poll() is None:
                        capture.kill()
                    capture.communicate(timeout=5)

    @unittest.skipUnless(os.name == "posix", "POSIX process groups")
    def test_capture_stops_workers_after_parent_exits(self):
        pid_file = self.root / "worker.pid"
        heartbeat = self.root / "heartbeat"
        worker = (
            "import os, signal, time\n"
            "from pathlib import Path\n"
            "signal.signal(signal.SIGINT, signal.SIG_IGN)\n"
            f"Path({str(pid_file)!r}).write_text(str(os.getpid()))\n"
            f"with open({str(heartbeat)!r}, 'ab', buffering=0) as output:\n"
            "    while True:\n"
            "        output.write(b'x')\n"
            "        time.sleep(0.02)\n"
        )
        parent = (
            "import subprocess, sys, time\n"
            f"subprocess.Popen([sys.executable, '-c', {worker!r}])\n"
            "time.sleep(30)\n"
        )
        output = self.root / "run"
        try:
            result = self.run_script(
                "capture_run.py", "--output-dir", output, "--timeout", "2", "--grace", "0.1",
                "--", sys.executable, "-c", parent,
            )
            self.assertEqual(result.returncode, 124, result.stderr)
            self.assertTrue(pid_file.exists(), "worker never started")
            manifest = json.loads((output / "manifest.json").read_text())
            self.assertFalse(manifest["child_still_running"])
            size = heartbeat.stat().st_size
            time.sleep(0.15)
            self.assertEqual(heartbeat.stat().st_size, size, "orphan worker is still running")
            worker_pid = int(pid_file.read_text())
            status = subprocess.run(
                ["ps", "-p", str(worker_pid), "-o", "stat="],
                capture_output=True, text=True, timeout=5,
            )
            self.assertIn(status.returncode, (0, 1), status.stderr)
            self.assertFalse(status.stderr.strip(), status.stderr)
            # An unreaped zombie is dead; kill(pid, 0) alone cannot distinguish it.
            state = status.stdout.strip()
            self.assertTrue(not state or state.startswith(("Z", "X")), f"worker {worker_pid} is alive: {state}")
            self.assertIn("terminate", manifest["termination_actions"])
            self.assertIn("process_group_present", manifest)
        finally:
            if pid_file.exists():
                try:
                    os.kill(int(pid_file.read_text()), signal.SIGKILL)
                except ProcessLookupError:
                    pass

    @unittest.skipUnless(os.name == "posix", "POSIX signal behavior")
    def test_capture_signal_exit_retains_raw_status(self):
        output = self.root / "run"
        result = self.run_script(
            "capture_run.py", "--output-dir", output, "--", sys.executable, "-c",
            "import os, signal; os.kill(os.getpid(), signal.SIGTERM)",
        )
        self.assertEqual(result.returncode, 128 + signal.SIGTERM)
        self.assertEqual(json.loads((output / "manifest.json").read_text())["returncode"], -signal.SIGTERM)

    def test_capture_rejects_unbounded_numeric_timeout(self):
        result = self.run_script(
            "capture_run.py", "--output-dir", self.root / "run",
            "--timeout", "nan", "--", sys.executable, "-c", "pass",
        )
        self.assertEqual(result.returncode, 2)
        self.assertFalse((self.root / "run").exists())

    def test_legacy_rvc2_json(self):
        artifact = self.root / "legacy.json"
        artifact.write_text(json.dumps({"depthaiCommitHash": "firmware-identity", "crashReports": [{}]}))
        result, report, output = self.inspect(artifact)
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertEqual(report["content"]["kind"], "rvc2_json")
        self.assertEqual(report["content"]["firmware_commit"], "firmware-identity")
        self.assertEqual((output / "crash_reports.json").read_bytes(), artifact.read_bytes())

    def test_excessive_json_nesting_is_reported_as_failed_inspection(self):
        nested = "[" * 20000 + "0" + "]" * 20000
        document = ('{"crashReports":[],"extra":' + nested + "}").encode()
        legacy = self.root / "deep.json"
        legacy.write_bytes(document)
        archive = self.archive("deep.tar", [("metadata.json", document)])
        # Some Python versions decode this depth but fail while encoding the report.
        metadata = ('{"extra":' + "[" * 2000 + "0" + "]" * 2000 + "}").encode()
        encoding = self.archive("deep-encoding.tar", [("metadata.json", metadata)])
        for index, artifact in enumerate((legacy, archive, encoding)):
            with self.subTest(format=index):
                result, report, _ = self.inspect(artifact, directory=f"deep-{index}")
                self.assertEqual(result.returncode, 2, result.stderr)
                self.assertEqual(report["status"], "failed")
                self.assertIn("recursion", report["error"].lower())

    def test_sdk_rvc2_preserves_host_and_firmware_identity(self):
        artifact = self.archive("rvc2.tar", [
            ("./metadata.json", b'{"platform":"RVC2","depthaiCommitHash":"host-identity"}'),
            ("crash_reports.json", b'{"depthaiCommitHash":"firmware-identity","crashReports":[{}]}'),
            ("extra.json", b"{}"),
        ])
        symbols = self.root / "unmatched.elf"
        symbols.write_bytes(b"unmatched symbols")
        result, report, _ = self.inspect(artifact, "--symbol-file", symbols)
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertEqual(report["content"]["metadata"]["depthaiCommitHash"], "host-identity")
        self.assertEqual(report["content"]["firmware_commit"], "firmware-identity")
        self.assertEqual(report["symbol_match"], "not_verified")

    def test_nested_rvc4_inventory_without_core_extraction(self):
        inner = self.archive("inner.tar.gz", [("core.123", b"synthetic core"), ("stdout.log", b"log")], "w:gz")
        artifact = self.archive("rvc4.tar", [
            ("metadata.json", b'{"platform":"RVC4"}'),
            ("crash_dump.tar.gz", inner.read_bytes()),
        ])
        before = artifact.read_bytes()
        result, report, output = self.inspect(artifact)
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertEqual(report["content"]["kind"], "rvc4_archive")
        self.assertEqual(report["content"]["nested"]["kind"], "core_candidate_archive")
        self.assertFalse(report["content"]["nested"]["core_candidates"][0]["elf_core_header"])
        self.assertTrue(report["content"]["nested"]["warnings"])
        self.assertFalse(list(output.rglob("core.123")))
        self.assertEqual(artifact.read_bytes(), before)

    def test_direct_gate_archive(self):
        artifact = self.archive("gate.tar", [("core.456", b"synthetic core")])
        result, report, _ = self.inspect(artifact)
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertEqual(report["content"]["kind"], "core_candidate_archive")

    def test_elf_core_identification(self):
        header = bytearray(64)
        header[:6] = b"\x7fELF\x02\x01"
        header[16:20] = struct.pack("<HH", 4, 183)
        artifact = self.root / "core"
        artifact.write_bytes(header)
        result, report, _ = self.inspect(artifact)
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertEqual(report["content"]["kind"], "elf_core")
        self.assertEqual(report["content"]["elf_machine"], 183)
        self.assertEqual(report["symbol_match"], "not_verified")

    def test_archive_core_header_is_inspected_without_extraction(self):
        header = bytearray(64)
        header[:6] = b"\x7fELF\x02\x01"
        header[16:20] = struct.pack("<HH", 4, 183)
        artifact = self.archive("gate.tar", [("core.123", bytes(header))])
        result, report, output = self.inspect(artifact)
        self.assertEqual(result.returncode, 0, result.stderr)
        candidate = report["content"]["core_candidates"][0]
        self.assertTrue(candidate["elf_core_header"])
        self.assertEqual(candidate["elf_machine"], 183)
        self.assertFalse((output / "core.123").exists())

    def test_rejects_unsafe_paths_links_and_duplicates(self):
        link = tarfile.TarInfo("metadata.json")
        link.type = tarfile.SYMTYPE
        link.linkname = "../outside"
        fixtures = [
            [("../escaped", b"x")],
            [("/absolute", b"x")],
            [("C:\\escaped", b"x")],
            [(link, b"")],
            [("metadata.json", b"{}"), ("./metadata.json", b"{}")],
        ]
        for index, members in enumerate(fixtures):
            with self.subTest(index=index):
                artifact = self.archive("bad" + str(index) + ".tar", members)
                result, report, _ = self.inspect(artifact, directory="bad" + str(index))
                self.assertEqual(result.returncode, 2)
                self.assertEqual(report["status"], "failed")
        self.assertFalse((self.root / "escaped").exists())

    def test_nested_archive_uses_same_size_budget(self):
        inner = self.archive("inner.tar.gz", [("core.1", b"x" * 4096)], "w:gz")
        artifact = self.archive("outer.tar", [("crash_dump.tar.gz", inner.read_bytes())])
        total_expanded = artifact.stat().st_size + len(gzip.decompress(inner.read_bytes()))
        result, report, _ = self.inspect(artifact, "--max-bytes", str(total_expanded - 1))
        self.assertEqual(result.returncode, 2)
        self.assertIn("max-bytes", report["error"])
        result, report, _ = self.inspect(
            artifact, "--max-bytes", str(total_expanded), directory="exact-budget",
        )
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertEqual(report["content"]["kind"], "rvc4_archive")

    def test_pax_metadata_obeys_budget_in_both_archive_layers(self):
        member = tarfile.TarInfo("stdout.log")
        member.pax_headers = {"comment": "x" * (1024 * 1024)}
        inner = self.archive("pax.tar.gz", [(member, b"x")], "w:gz")
        outer = self.archive("outer.tar", [("crash_dump.tar.gz", inner.read_bytes())])
        for index, artifact in enumerate((inner, outer)):
            with self.subTest(layer=index):
                result, report, _ = self.inspect(
                    artifact, "--max-bytes", str(64 * 1024), directory=f"pax-{index}",
                )
                self.assertEqual(result.returncode, 2)
                self.assertIn("max-bytes", report["error"])

    def test_gnu_longname_obeys_budget(self):
        artifact = self.archive(
            "longname.tar.gz", [("x" * (128 * 1024), b"x")],
            "w:gz", format=tarfile.GNU_FORMAT,
        )
        result, report, _ = self.inspect(artifact, "--max-bytes", str(64 * 1024))
        self.assertEqual(result.returncode, 2)
        self.assertIn("max-bytes", report["error"])

    def test_compressed_archives_with_small_pax_headers_remain_supported(self):
        for index, mode in enumerate(("w", "w:gz", "w:bz2", "w:xz")):
            with self.subTest(mode=mode):
                member = tarfile.TarInfo("metadata.json")
                member.pax_headers = {"comment": "preserved metadata"}
                artifact = self.archive(
                    f"compressed-{index}.tar", [
                        (member, b'{"platform":"RVC2"}'),
                        ("crash_reports.json", b'{"crashReports":[{}]}'),
                    ], mode,
                )
                result, report, _ = self.inspect(
                    artifact, "--max-bytes", str(64 * 1024), directory=f"compression-{index}",
                )
                self.assertEqual(result.returncode, 0, result.stderr)
                self.assertEqual(report["content"]["kind"], "rvc2_archive")

    def test_corrupt_compression_is_reported_as_failed_inspection(self):
        for index, signature in enumerate((b"\x1f\x8b", b"BZh", b"\xfd7zXZ\x00")):
            with self.subTest(signature=signature):
                artifact = self.root / f"broken-{index}.tar"
                artifact.write_bytes(signature + b"invalid")
                result, report, _ = self.inspect(artifact, directory=f"broken-{index}")
                self.assertEqual(result.returncode, 2, result.stderr)
                self.assertEqual(report["status"], "failed")

    def test_corrupt_artifact_and_conflicting_platform(self):
        corrupt = self.root / "broken.tar"
        corrupt.write_bytes(b"not a tar archive")
        result, report, _ = self.inspect(corrupt)
        self.assertEqual(result.returncode, 2)
        self.assertEqual(report["status"], "failed")
        conflict = self.archive("conflict.tar", [
            ("metadata.json", b'{"platform":"RVC4"}'),
            ("crash_reports.json", b'{"crashReports":[{}]}'),
        ])
        result, report, _ = self.inspect(conflict, directory="conflict")
        self.assertEqual(result.returncode, 2)
        self.assertIn("conflict", report["error"])

    def test_inspector_never_overwrites(self):
        artifact = self.root / "legacy.json"
        artifact.write_text('{"crashReports":[]}')
        output = self.root / "inspection"
        output.mkdir()
        (output / "inspection.json").write_text("preserved")
        result, _, _ = self.inspect(artifact, directory="existing-inspection")
        self.assertEqual(result.returncode, 0)
        result = self.run_script("inspect_crashdump.py", artifact, "--output-dir", output)
        self.assertEqual(result.returncode, 2)
        self.assertEqual((output / "inspection.json").read_text(), "preserved")


if __name__ == "__main__":
    unittest.main()
