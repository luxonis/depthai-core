import json
from pathlib import Path
import re
import shutil
import subprocess
import sys
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[2]


@unittest.skipUnless(shutil.which("cmake") and shutil.which("ctest"), "CMake/CTest required")
class FfcSelectionTest(unittest.TestCase):
    def setUp(self):
        self.workspace = tempfile.TemporaryDirectory()
        self.addCleanup(self.workspace.cleanup)
        self.root = Path(self.workspace.name)
        self.build = self.root / "build"
        source = (ROOT / "tests/CMakeLists.txt").read_text()
        start = source.index("function(dai_set_test_labels test_name)")
        end = source.index("endfunction()", start) + len("endfunction()")
        definitions = source[start:end]
        for name in ["logging_test", "script_node_rvc4_test", "stereo_depth_node_test", "system_logger_test"]:
            call = re.search(rf"^dai_set_test_labels\({name}\s+.*\)$", source, re.MULTILINE).group()
            definitions += f"\nadd_test(NAME {name} COMMAND ${{CMAKE_COMMAND}} -E true)\n{call}\n"
        definitions += """
add_test(NAME rvc4_only COMMAND ${CMAKE_COMMAND} -E true)
dai_set_test_labels(rvc4_only ondevice rvc4 ci)
add_test(NAME ffc_only COMMAND ${CMAKE_COMMAND} -E true)
dai_set_test_labels(ffc_only ondevice rvc4ffc ci)
"""
        self.cmake = self.root / "CMakeLists.txt"
        self.cmake.write_text(
            "cmake_minimum_required(VERSION 3.20)\nproject(Labels NONE)\nenable_testing()\n" + definitions
        )
        self.configure()

    def configure(self):
        subprocess.run(["cmake", "-S", str(self.root), "-B", str(self.build)], check=True, capture_output=True)

    def run_ffc(self):
        return subprocess.run(
            [sys.executable, str(ROOT / "tests/run_tests.py"), "--rvc4ffc", "--test_dir", str(self.build)],
            text=True, capture_output=True, timeout=30,
        )

    def test_labels_are_explicit_and_independent(self):
        for label, included, excluded in [("rvc4ffc", "ffc_only", "rvc4_only"), ("rvc4", "rvc4_only", "ffc_only")]:
            data = json.loads(subprocess.check_output(
                ["ctest", "--test-dir", str(self.build), "--show-only=json-v1", "-L", f"^{label}$", "-L", "^ci$"],
                text=True,
            ))
            names = {test["name"] for test in data["tests"]}
            self.assertEqual(len(names), 5)
            self.assertIn(included, names)
            self.assertNotIn(excluded, names)

    def test_cli_runs_only_ffc_selection(self):
        result = self.run_ffc()
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
        self.assertIn("ffc_only", result.stdout)
        self.assertNotIn("rvc4_only", result.stdout)
        self.assertIn("RVC4 - FFC", result.stdout)

    def test_cli_preserves_test_failures(self):
        self.cmake.write_text(self.cmake.read_text().replace(
            "NAME system_logger_test COMMAND ${CMAKE_COMMAND} -E true",
            "NAME system_logger_test COMMAND ${CMAKE_COMMAND} -E false",
        ))
        self.configure()
        result = self.run_ffc()
        self.assertEqual(result.returncode, 1, result.stdout + result.stderr)
        self.assertIn("Some tests failed", result.stdout)


if __name__ == "__main__":
    unittest.main()
