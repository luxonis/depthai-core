import contextlib
import io
import runpy
import sys
import tempfile
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock, patch

from ctest_to_junit import parse_log, write_junit


class RecordingPoint:
    def __init__(self, measurement):
        self.measurement = measurement
        self.fields = {}
        self.tags = {}

    def time(self, value):
        return self

    def tag(self, name, value):
        self.tags[name] = value
        return self

    def field(self, name, value):
        self.fields[name] = value
        return self


class InfluxReportTests(unittest.TestCase):
    def upload(self, log, previous=()):
        influx = Mock()
        influx.Point.side_effect = RecordingPoint
        client = influx.InfluxDBClient.return_value
        client.query_api.return_value.query.return_value = [
            SimpleNamespace(records=[{"testName": name} for name in previous])
        ]
        with tempfile.TemporaryDirectory() as directory:
            source = Path(directory) / "ctest.log"
            target = Path(directory) / "junit.xml"
            source.write_text(log)
            write_junit(target, "job", *parse_log(source))
            with patch.dict(sys.modules, {
                "influxdb_client": influx,
                "influxdb_client.client": Mock(),
                "influxdb_client.client.write_api": Mock(SYNCHRONOUS=None),
            }), patch.object(sys, "argv", ["uploader", "2026-10-22T00:00:00", "linux", str(target)]), contextlib.redirect_stdout(io.StringIO()):
                runpy.run_path(str(Path(__file__).with_name("junit_to_influxdb.py")))
        return [call.kwargs["record"] for call in client.write_api.return_value.write.call_args_list]

    def test_missing_results_are_uploaded_as_errors_without_fixing_old_failures(self):
        for log in ("No runner could be started\n", "[USB] 17: setup failed before any results\n"):
            with self.subTest(log=log):
                points = self.upload(log, previous=("camera",))
                testcase, suite = points
                self.assertEqual(testcase.measurement, "test")
                self.assertEqual(testcase.fields["status"], 0)  # errored
                self.assertIn("CTest", testcase.fields["logs"])
                self.assertEqual(suite.measurement, "testsuite")
                for key, value in {"tests": 1, "errored": 1, "failed": 0, "passed": 0, "fixed": 0}.items():
                    self.assertEqual(suite.fields[key], value)

    def test_only_an_observed_pass_fixes_a_previous_failure(self):
        points = self.upload("""[USB] 1/1 Test #1: camera .......... Passed 0.10 sec
[USB] 100% tests passed, 0 tests failed out of 1
""", previous=("camera", "not_run"))
        self.assertEqual(points[0].fields["status"], 3)  # passed
        self.assertEqual(points[1].fields["fixed"], 1)
        self.assertEqual(points[1].fields["errored"], 0)

    def test_skipped_test_does_not_fix_a_previous_failure(self):
        points = self.upload("""[USB] 1/1 Test #1: camera ..........***Skipped 0.10 sec
[USB] 100% tests passed, 0 tests failed out of 1
""", previous=("camera",))
        self.assertEqual(points[0].fields["status"], 2)  # skipped
        self.assertEqual(points[1].fields["skipped"], 1)
        self.assertEqual(points[1].fields["fixed"], 0)

    def test_disabled_tests_do_not_reduce_passes_or_total(self):
        points = self.upload("""[USB] 1/4 Test #1: camera .......... Passed 0.10 sec
[USB] 2/4 Test #2: disabled ..........***Not Run (Disabled) 0.00 sec
[USB] 3/4 Test #3: skipped ..........***Skipped 0.10 sec
[USB] 4/4 Test #4: failed ..........***Failed 0.10 sec
[USB] 67% tests passed, 1 tests failed out of 3
""")
        suite = points[-1]
        for key, value in {"tests": 4, "passed": 1, "failed": 1, "skipped": 2, "errored": 0}.items():
            self.assertEqual(suite.fields[key], value)


if __name__ == "__main__":
    unittest.main()
