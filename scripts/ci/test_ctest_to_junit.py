import tempfile
import unittest
from pathlib import Path
from xml.etree import ElementTree as ET

import ctest_to_junit


class CTestToJUnitTest(unittest.TestCase):
    def report(self, log):
        with tempfile.TemporaryDirectory() as directory:
            log_path = Path(directory) / "ctest.log"
            junit_path = Path(directory) / "junit.xml"
            log_path.write_text(log)
            parsed = ctest_to_junit.parse_log(log_path)
            ctest_to_junit.write_junit(junit_path, "linux", *parsed)
            return ET.parse(junit_path).getroot()

    def test_announced_configuration_without_tests_is_incomplete(self):
        for labels in ("ci", ""):
            with self.subTest(labels=labels):
                report = self.report(
                    "Running tests for configuration: rvc4, on platform: RVC4, "
                    f"on protocol: TCPIP, with labels: {labels}\n"
                )
                suite = report.find("testsuite")
                self.assertEqual(suite.get("tests"), "1")
                self.assertEqual(suite.get("failures"), "1")
                self.assertEqual(suite.get("DEPTHAI_PLATFORM"), "RVC4")
                self.assertEqual(suite.get("labels"), labels)
                self.assertIsNotNone(
                    suite.find("testcase[@name='#0 ctest_incomplete']/failure")
                )

    def test_crashed_test_and_unfinished_test_remain_failures(self):
        report = self.report(
            "[rvc4] Start 1: crashed\n"
            "[rvc4] 1/3 Test #1: crashed ........***Exception: SegFault 0.10 sec\n"
            "[rvc4] Start 2: unfinished\n"
        )
        suite = report.find("testsuite")
        self.assertEqual(suite.get("tests"), "3")
        self.assertEqual(suite.get("failures"), "3")
        self.assertEqual(
            suite.find("testcase[@name='#1 crashed']/failure").get("message"),
            "Exception: SegFault",
        )
        self.assertEqual(
            suite.find("testcase[@name='#2 unfinished']/failure").get("message"),
            "Did not finish",
        )
        self.assertIn(
            "1 of 3 planned tests did not start",
            suite.find("testcase[@name='#0 ctest_incomplete']/failure").text,
        )

    def test_completed_run_preserves_passes_skips_and_failures(self):
        report = self.report(
            "Running tests for configuration: rvc4, on platform: RVC4, "
            "on protocol: TCPIP, with labels: ci\n"
            "[rvc4] 1/3 Test #1: passed ........ Passed 0.10 sec\n"
            "[rvc4] 2/3 Test #2: skipped ........***Skipped 0.10 sec\n"
            "[rvc4] 3/3 Test #3: failed ........***Failed 0.10 sec\n"
            "[rvc4] 67% tests passed, 1 tests failed out of 3\n"
        )
        suite = report.find("testsuite")
        self.assertEqual(suite.get("tests"), "3")
        self.assertEqual(suite.get("failures"), "1")
        self.assertEqual(suite.get("skipped"), "1")
        self.assertIsNone(suite.find("testcase[@name='#0 ctest_incomplete']"))


if __name__ == "__main__":
    unittest.main()
