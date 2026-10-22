import subprocess
import sys
import tempfile
import unittest
from pathlib import Path
from xml.etree import ElementTree as ET

from ctest_to_junit import parse_log, write_junit


class CTestReportTests(unittest.TestCase):
    def report(self, log):
        with tempfile.TemporaryDirectory() as directory:
            source = Path(directory) / "ctest.log"
            target = Path(directory) / "junit.xml"
            source.write_text(log)
            write_junit(target, "job", *parse_log(source))
            return ET.parse(target).getroot()

    def test_skip_is_not_a_pass_or_failure(self):
        root = self.report("""[RVC4 - USB] 1/3 Test #1: passed ..............   Passed 1.00 sec
[RVC4 - USB] 2/3 Test #2: skipped .............***Skipped 0.02 sec
[RVC4 - USB] 3/3 Test #3: failed ..............***Failed 0.03 sec
[RVC4 - USB] 67% tests passed, 1 tests failed out of 3
[RVC4 - USB] The following tests did not run:
[RVC4 - USB] 2 - skipped (Skipped)
[RVC4 - USB] The following tests FAILED:
[RVC4 - USB] 3 - failed (Failed)
""")
        self.assertEqual(root.get("tests"), "3")
        self.assertEqual(root.get("failures"), "1")
        self.assertEqual(root.get("skipped"), "1")
        self.assertEqual(len(root.findall(".//testcase")), 3)
        self.assertEqual(len(root.findall(".//skipped")), 1)
        self.assertEqual(root.find("testsuite").get("errors"), "0")

    def test_cancelled_log_retains_failures_without_summary(self):
        root = self.report("""[RVC4] 17: firmware disconnected
[RVC4] 1/4 Test #17: camera ..........***Failed 2.00 sec
[RVC4] 2/4 Test #18: sync ............Subprocess aborted***Exception: 0.10 sec
[RVC4] 3/4 Test #19: missing .........***Not Run 0.00 sec
""")
        # Develop also reports an explicit failure for the incomplete run.
        self.assertEqual(root.get("failures"), "4")
        self.assertEqual(root.get("tests"), "4")
        self.assertIn("firmware disconnected", root.find(".//failure").text)
        cases = {case.get("name"): case for case in root.findall(".//testcase")}
        self.assertIn("Subprocess aborted", cases["#18 sync"].find("failure").get("message"))
        self.assertIn("#0 ctest_incomplete", cases)

    def test_all_skipped_suite_has_no_passes(self):
        root = self.report("""[USB] 1/1 Test #122: crashdump_test ........***Skipped 7.58 sec
[USB] 100% tests passed, 0 tests failed out of 1
[USB] The following tests did not run:
[USB] 122 - crashdump_test (Skipped)
""")
        self.assertEqual(root.get("tests"), "1")
        self.assertEqual(root.get("failures"), "0")
        self.assertEqual(root.get("skipped"), "1")
        summary = root.find(".//property").get("value")
        self.assertEqual(summary, "Passed=0, Failed=0, Total=1")

    def test_empty_log_is_an_error(self):
        root = self.report("No test runner could be started\n")
        self.assertEqual(root.get("errors"), "1")
        self.assertIsNotNone(root.find(".//error"))

    def test_configuration_output_without_results_is_an_error(self):
        root = self.report("[USB] 17: setup failed before any results\n")
        self.assertEqual(root.get("errors"), "1")
        self.assertEqual(root.get("tests"), "1")
        self.assertIsNotNone(root.find(".//error"))

    def test_repeated_test_uses_its_final_outcome(self):
        root = self.report("""[USB] 1/1 Test #1: repeated ..........***Skipped 0.00 sec
[USB] 1/1 Test #1: repeated ..........***Failed 0.10 sec
[USB] 1/1 Test #1: repeated .......... Passed 0.10 sec
[USB] 100% tests passed, 0 tests failed out of 1
""")
        self.assertEqual(root.get("tests"), "1")
        self.assertEqual(root.get("failures"), "0")
        self.assertEqual(root.get("skipped"), "0")
        self.assertEqual(len(root.findall(".//testcase")), 1)

    def test_announced_configuration_remains_an_incomplete_failure(self):
        root = self.report("Running tests for configuration: RVC4, on platform: rvc4, on protocol: usb, with labels: ci\n")
        self.assertEqual(root.get("failures"), "1")
        self.assertEqual(root.find(".//testcase").get("name"), "#0 ctest_incomplete")

    def test_started_test_without_result_remains_a_failure(self):
        root = self.report("""[USB] Start 1: camera
[USB] 1: camera stopped responding
""")
        self.assertEqual(root.get("failures"), "2")
        failure = root.find(".//failure")
        self.assertEqual(failure.get("message"), "Did not finish")
        self.assertIn("camera stopped responding", failure.text)

    def test_test_output_is_not_a_failed_test_list(self):
        root = self.report("""[USB] 1 - misleading_output (Failed)
[USB] 1/1 Test #1: passed .......... Passed 0.10 sec
[USB] 100% tests passed, 0 tests failed out of 1
""")
        self.assertEqual(root.get("failures"), "0")
        self.assertEqual(root.get("tests"), "1")

    def test_disabled_and_unpadded_results_remain_supported(self):
        root = self.report("""[USB] 1/2 Test #1: a long test name   Passed 0.10 sec
[USB] 2/2 Test #2: disabled ..........***Not Run (Disabled) 0.00 sec
[USB] 100% tests passed, 0 tests failed out of 1
""")
        self.assertEqual(root.get("tests"), "2")
        self.assertEqual(root.get("failures"), "0")
        self.assertEqual(root.get("skipped"), "1")
        self.assertEqual(root.find(".//property").get("value"), "Passed=1, Failed=0, Total=2")
        self.assertEqual(root.find(".//testcase/success/..").get("name"), "#1 a long test name")

    def test_all_disabled_suite_does_not_require_a_summary(self):
        root = self.report("""[USB] Start 1: disabled
[USB] 1/1 Test #1: disabled ..........***Not Run (Disabled) 0.00 sec
[USB] No tests were found!!!
""")
        self.assertEqual(root.get("tests"), "1")
        self.assertEqual(root.get("failures"), "0")
        self.assertEqual(root.get("skipped"), "1")
        self.assertEqual(root.find(".//property").get("value"), "Passed=0, Failed=0, Total=1")

    def test_partial_disabled_results_remain_incomplete(self):
        root = self.report("[USB] 1/2 Test #1: disabled ..........***Not Run (Disabled) 0.00 sec\n")
        self.assertEqual(root.get("failures"), "1")
        self.assertEqual(root.find(".//failure").get("message"), "CTest run did not complete")

    def test_subprocess_in_a_test_name_does_not_change_its_outcome(self):
        for status, failed in (("Passed", 0), ("Subprocess aborted***Exception:", 1)):
            with self.subTest(status=status):
                root = self.report(f"""[USB] 1/1 Test #1: a Subprocess worker ..........{status} 0.10 sec
[USB] {100 * (1 - failed)}% tests passed, {failed} tests failed out of 1
""")
                self.assertEqual(root.get("failures"), str(failed))
                self.assertEqual(root.find(".//testcase").get("name"), "#1 a Subprocess worker")

    def test_skip_totals_include_all_configurations(self):
        root = self.report("""[USB] 1/1 Test #1: skipped ..........***Skipped 0.00 sec
[USB] 100% tests passed, 0 tests failed out of 1
[POE] 1/1 Test #2: skipped ..........***Skipped 0.00 sec
[POE] 100% tests passed, 0 tests failed out of 1
""")
        self.assertEqual(root.get("tests"), "2")
        self.assertEqual(root.get("skipped"), "2")
        self.assertEqual(root.get("errors"), "0")

    def test_markdown_summary_keeps_skips_and_unfinished_tests_separate(self):
        with tempfile.TemporaryDirectory() as directory:
            source = Path(directory) / "ctest.log"
            source.write_text("""[complete] 1/3 Test #1: passed .......... Passed 0.10 sec
[complete] 2/3 Test #2: skipped ..........***Skipped 0.00 sec
[complete] 3/3 Test #3: disabled ..........***Not Run (Disabled) 0.00 sec
[complete] 100% tests passed, 0 tests failed out of 2
[partial] 1/2 Test #1: passed .......... Passed 0.10 sec
[partial] Start 2: camera
[partial] 2: camera stopped responding
""")
            summary = subprocess.run(
                [sys.executable, str(Path(__file__).with_name("ctest_summary.py")), str(source)],
                check=True, capture_output=True, text=True,
            ).stdout
        self.assertIn("| complete | 1 | 0 | 2 | 3 |", summary)
        self.assertIn("| partial (incomplete, no ctest summary) | 1 | 1 | 0 | 2 |", summary)
        self.assertIn("#2 camera - Did not finish", summary)
        self.assertIn("camera stopped responding", summary)


if __name__ == "__main__":
    unittest.main()
