"""Behavioral tests for the debugging helpers; no SDK import, build or device."""

import hashlib
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

    def archive(self, filename, members, mode="w"):
        path = self.root / filename
        with tarfile.open(path, mode) as archive:
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
        result, report, _ = self.inspect(artifact, "--max-bytes", str(inner.stat().st_size + 4095))
        self.assertEqual(result.returncode, 2)
        self.assertIn("max-bytes", report["error"])

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
