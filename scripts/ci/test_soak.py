#!/usr/bin/env python3
"""Checks soak_summary.py and soak_compare.py on trimmed logs of real hil_testing_nightly.yml runs.

Run with: python3 -m unittest discover -s scripts/ci -p test_soak.py

The fixtures keep every ctest result line, the hil_runner lines the summary reads, the no-device
lines and the output of failed tests. IP addresses and testbed names are replaced.
- rvc4_completed.log: run 43750054123, Ethernet row, 191 tests passed.
- rvc4usb_completed.log: run 43750054123, USB row, 189 tests passed.
- rvc4usb_lost_device.log: run 43285928756, USB row. hfr_small_preview failed, the camera was gone
  afterwards and 98 tests skipped with "No device available".
- rvc4_cancelled.log: run 43248339227, Ethernet row, cancelled during its ninth test.
- rvc4usb_fsync_failed.log: run 43859876832 of hil_aux_soak.yml, USB row, only fsync_test and the ctest
  summary. Its Catch2 failure is followed by 36 lines of benchmark warnings.
"""
import contextlib
import io
import json
import sqlite3
import tempfile
import unittest
from pathlib import Path

import soak_compare
import soak_summary

FIXTURES = Path(__file__).resolve().parent / "soak_fixtures"
AUX_OS = "aux-con-dev-4+9905f59d3949db3459296f554d5e15145ad585a3"


def write_pass(root, os, row, number, log=None, **meta):
    directory = root / f"{row}-{os}-p{number}"
    directory.mkdir(parents=True)
    (directory / "pass.json").write_text(json.dumps({"os": os, "row": row, "pass": number, **meta}))
    if log is not None:
        (directory / "soak.log").write_text(log)


def fixture(name):
    return (FIXTURES / name).read_text()


def summarize(root, os_versions, rows, passes, depthai_ref="feat/aux-controller-integration", run_id=1,
              night="2026-11-17"):
    out = root / f"results-{run_id}.json"
    stdout = io.StringIO()
    with contextlib.redirect_stdout(stdout):
        soak_summary.main([
            str(root / "passes"), "--out", str(out), "--night", night, "--run-id", str(run_id),
            "--run-attempt", "1", "--soak-ref", "ci/aux-soak-aux", "--depthai-ref", depthai_ref,
            "--depthai-sha", "0" * 40, "--device-pin", "0.0.1+test", "--os-versions", json.dumps(os_versions),
            "--rows", json.dumps(rows), "--passes", str(passes),
        ])
    return json.loads(out.read_text()), stdout.getvalue(), out


def outcomes(results, row, number):
    return {r["test"]: r["outcome"] for r in results["records"] if r["row"] == row and r["pass"] == number}


def pass_of(results, row, number):
    return next(p for p in results["passes"] if p["row"] == row and p["pass"] == number)


class SummaryTest(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.root = Path(self.tmp.name)
        self.passes = self.root / "passes"

    def tearDown(self):
        self.tmp.cleanup()

    def test_completed_ethernet_pass(self):
        write_pass(self.passes, "1.44.0", "rvc4", 1, fixture("rvc4_completed.log"))
        results, _, _ = summarize(self.root, ["1.44.0"], ["rvc4"], 1)

        tests = outcomes(results, "rvc4", 1)
        self.assertEqual(len(tests), 191)
        self.assertEqual(set(tests.values()), {"passed"})
        summary = pass_of(results, "rvc4", 1)
        self.assertEqual(summary["pass_outcome"], "completed")
        self.assertEqual(summary["testbed"], "testbed-a1-4pro")
        self.assertEqual(summary["pairing"], "aux+1.44.0")

    def test_lost_device_is_flaky_failure_and_mass_skip(self):
        write_pass(self.passes, AUX_OS, "rvc4usb", 1, fixture("rvc4usb_lost_device.log"))
        write_pass(self.passes, AUX_OS, "rvc4usb", 2, fixture("rvc4usb_completed.log"))
        write_pass(self.passes, AUX_OS, "rvc4usb", 3, fixture("rvc4usb_completed.log"))
        results, page, _ = summarize(self.root, [AUX_OS], ["rvc4usb"], 3)

        lost = outcomes(results, "rvc4usb", 1)
        self.assertEqual(lost["hfr_small_preview"], "failed")
        self.assertEqual(sum(o == "no_device" for o in lost.values()), 98)
        self.assertEqual(outcomes(results, "rvc4usb", 2)["hfr_small_preview"], "passed")
        failed = next(r for r in results["records"] if r["outcome"] == "failed")
        self.assertIn("produced an error (6) while running", failed["failure"])
        self.assertEqual(failed["seconds"], 72.2)
        self.assertEqual([pass_of(results, "rvc4usb", n)["mass_skip"] for n in (1, 2, 3)], [True, False, False])
        self.assertEqual(pass_of(results, "rvc4usb", 1)["pairing"], "aux+aux-con-dev-4")
        flaky = page.split("### Flaky tests")[1].split("###")[0]
        self.assertIn("| aux+aux-con-dev-4 | rvc4usb | hfr_small_preview | 2 | 1 | 0 |", flaky)

    def test_catch2_failure_keeps_the_assertion_and_the_time(self):
        write_pass(self.passes, AUX_OS, "rvc4usb", 1, fixture("rvc4usb_fsync_failed.log"))
        results, _, _ = summarize(self.root, [AUX_OS], ["rvc4usb"], 1)

        failed = next(r for r in results["records"] if r["test"] == "fsync_test")
        self.assertEqual(failed["outcome"], "failed")
        self.assertEqual(failed["seconds"], 49.58)
        self.assertEqual(failed["failure"], "\n".join([
            "Cause: Failed",
            "Test Fsync with different FPS values",
            "/workspace/tests/src/ondevice_tests/pipeline/node/fsync_test.cpp:269: FAILED:",
            "  REQUIRE( diffAll <= thresholds.all )",
            "with expansion:",
            "  1014533 ns <= 1000000 ns",
            "with messages:",
            "  fps := 10.0f",
            "  i := 0",
        ]))

    def test_cancelled_pass_is_partial_and_missing_tests_are_not_run(self):
        write_pass(self.passes, "1.44.0", "rvc4", 1, fixture("rvc4_completed.log"))
        write_pass(self.passes, "1.44.0", "rvc4", 2, fixture("rvc4_cancelled.log"))
        results, _, _ = summarize(self.root, ["1.44.0"], ["rvc4"], 2)

        self.assertEqual(pass_of(results, "rvc4", 2)["pass_outcome"], "partial")
        cancelled = outcomes(results, "rvc4", 2)
        self.assertEqual(len(cancelled), 191)
        self.assertEqual(sum(o == "passed" for o in cancelled.values()), 8)
        self.assertEqual(cancelled["dynamic_calibration_test"], "not_run")

    def test_not_started_and_missing_passes(self):
        write_pass(self.passes, "1.44.0", "rvc4", 1, fixture("rvc4_completed.log"))
        write_pass(self.passes, "1.44.0", "rvc4", 2, not_started="after 2026-11-17T04:30:00Z")
        results, page, _ = summarize(self.root, ["1.44.0"], ["rvc4"], 3)

        self.assertEqual([pass_of(results, "rvc4", n)["pass_outcome"] for n in (1, 2, 3)],
                         ["completed", "not_started", "infra"])
        self.assertEqual(set(outcomes(results, "rvc4", 3).values()), {"not_run"})
        self.assertEqual(len(results["records"]), 3 * 191)
        self.assertIn("no pass artifact", page)

    def test_testbed_change_after_failed_sanity(self):
        # hil_framework's ConfigFinder log lines for a testbed that failed its sanity check.
        failed_sanity = (
            "2026-11-17 01:10:00,000 [INFO] Checking sanity testbed-z1-4pro\n"
            "2026-11-17 01:14:00,000 [ERROR] Sanity check failed for testbed-z1-4pro. Rewriting reservation to "
            "endless 'blocked' and continuing search by policy.\n"
            "Sanity check failed for testbed-z1-4pro. Searching for another testbed.\n"
        )
        write_pass(self.passes, "1.44.0", "rvc4", 1, failed_sanity + fixture("rvc4_completed.log"))
        results, page, _ = summarize(self.root, ["1.44.0"], ["rvc4"], 1)

        summary = pass_of(results, "rvc4", 1)
        self.assertEqual(summary["testbeds_tried"], ["testbed-z1-4pro", "testbed-a1-4pro"])
        self.assertEqual(summary["testbed_changes"], 1)
        self.assertEqual(summary["testbed"], "testbed-a1-4pro")
        self.assertIn("testbed changed after a failed sanity check", page)

    def test_exit_125_counted_as_passed_is_no_device(self):
        # ExecuteTestTimeout.cmake accepts exit code 125; ctest calls it Passed when no skip regex matches.
        log = (
            "Running tests for configuration: RVC4, on platform: rvc4, on protocol: tcpip, with labels: rvc4\n"
            "[RVC4] 12: -- After process executed,  produced the following exit code: 125\n"
            "[RVC4] 12: -- No device available,  skipped\n"
            "[RVC4]  1/2 Test  #12: some_example .....................   Passed   31.00 sec\n"
            "[RVC4]  2/2 Test  #13: other_example ....................   Passed    2.00 sec\n"
            "[RVC4] 100% tests passed, 0 tests failed out of 2\n"
        )
        write_pass(self.passes, "1.44.0", "rvc4", 1, log)
        results, _, _ = summarize(self.root, ["1.44.0"], ["rvc4"], 1)

        self.assertEqual(outcomes(results, "rvc4", 1), {"some_example": "no_device", "other_example": "passed"})
        self.assertEqual(pass_of(results, "rvc4", 1)["no_device_as_pass"], 1)

    def test_release_tag_pairing(self):
        write_pass(self.passes, AUX_OS, "rvc4", 1, fixture("rvc4_completed.log"))
        results, _, _ = summarize(self.root, [AUX_OS], ["rvc4"], 1, depthai_ref="v3.12.1")

        self.assertEqual(pass_of(results, "rvc4", 1)["pairing"], "3.12.1+aux-con-dev-4")


class CompareTest(unittest.TestCase):
    PAIRINGS = ("3.12.1+aux-con-dev-4", "aux+aux-con-dev-4", "aux+1.44.0")

    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.root = Path(self.tmp.name)

    def tearDown(self):
        self.tmp.cleanup()

    def soak_run(self, run_id, night, depthai_ref, os_versions, first_pass_log):
        root = self.root / str(run_id)
        for os in os_versions:
            write_pass(root / "passes", os, "rvc4usb", 1, fixture(first_pass_log))
            write_pass(root / "passes", os, "rvc4usb", 2, fixture("rvc4usb_completed.log"))
        return summarize(root, os_versions, ["rvc4usb"], 2, depthai_ref=depthai_ref, run_id=run_id, night=night)[2]

    def test_failure_only_with_aux_depthai_points_to_the_depthai_change(self):
        files = []
        for run_id, night in ((10, "2026-11-17"), (20, "2026-11-18")):
            files.append(self.soak_run(run_id, night, "v3.12.1", [AUX_OS], "rvc4usb_completed.log"))
            files.append(self.soak_run(run_id + 1, night, "feat/aux-controller-integration", [AUX_OS, "1.44.0"],
                                       "rvc4usb_lost_device.log"))
        con = soak_compare.connect(":memory:")
        for path in files + files:
            soak_compare.ingest(con, path)

        self.assertEqual(con.execute("SELECT COUNT(*) FROM runs").fetchone()[0], 4)
        self.assertEqual(con.execute("SELECT COUNT(*) FROM results").fetchone()[0], 2 * 189 * 2 * 3)
        nights = soak_compare.window(con, 14)
        self.assertEqual(nights, ["2026-11-17", "2026-11-18"])
        hfr = next(c for c in soak_compare.compare(con, nights, self.PAIRINGS, 2) if c.test == "hfr_small_preview")
        self.assertEqual(hfr.failures, {self.PAIRINGS[0]: (0, 4), self.PAIRINGS[1]: (2, 4), self.PAIRINGS[2]: (2, 4)})
        self.assertEqual(hfr.flaky_nights, 2)
        self.assertEqual(hfr.reading, "The DepthAI change")
        page = soak_compare.report(con, 14, self.PAIRINGS, 2)
        self.assertIn("| 2026-11-18 | aux+1.44.0 | 21 | 2/2 | 1 | 1 | 1 | 0/1/0 |", page)

    def test_history_file_survives_reopening(self):
        path = self.soak_run(30, "2026-11-19", "v3.12.1", [AUX_OS], "rvc4usb_completed.log")
        db = self.root / "history.sqlite"
        soak_compare.ingest(soak_compare.connect(db), path)

        self.assertEqual(sqlite3.connect(db).execute("SELECT night FROM runs").fetchall(), [("2026-11-19",)])


if __name__ == "__main__":
    unittest.main()
