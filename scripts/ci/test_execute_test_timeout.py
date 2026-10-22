import shutil
import subprocess
import sys
import tempfile
import unittest
from pathlib import Path


@unittest.skipUnless(shutil.which("cmake"), "CMake is required")
class ExecuteTestTimeoutTests(unittest.TestCase):
    def test_environment_and_command_arguments_are_preserved(self):
        wrapper = Path(__file__).resolve().parents[2] / "examples/cpp/cmake/ExecuteTestTimeout.cmake"
        with tempfile.TemporaryDirectory() as directory:
            probe = Path(directory) / "probe.py"
            probe.write_text(r'''import os
import sys
assert os.environ["PATH"] == sys.argv[1]
assert os.environ["PYTHONPATH"] == sys.argv[2]
assert sys.argv[3:-1] == ["first argument", "value;with;semicolons", "plain argument", "", '"${literal}"']
raise SystemExit(int(sys.argv[-1]))
''')
            for path, python_path in (
                (r"C:\Program Files\CMake\bin;C:\Windows\System32", r"C:\modules;D:\other modules"),
                ("C:\\Windows\\", "C:\\modules\\"),
                ("C:\\first\\;D:\\second\\", "C:\\first;D:\\second\\"),
            ):
                for exit_code in (0, 7):
                    with self.subTest(path=path, python_path=python_path, exit_code=exit_code):
                        result = subprocess.run([
                            "cmake", "-DFORCE_TIMEOUT_SECONDS=5", "-P", str(wrapper),
                            "cmake", "-E", "env", f"PATH={path}", f"PYTHONPATH={python_path}",
                            sys.executable, str(probe), path, python_path,
                            "first argument", "value;with;semicolons", "plain argument", "", '"${literal}"', str(exit_code),
                        ], capture_output=True, text=True, timeout=10)
                        self.assertEqual(result.returncode == 0, exit_code == 0, result.stdout + result.stderr)
                        if exit_code:
                            self.assertIn("error (7)", result.stderr)


if __name__ == "__main__":
    unittest.main()
