#!/usr/bin/env python3
import re
import sys
from collections import OrderedDict
from pathlib import Path
from xml.etree import ElementTree as ET


ANSI_RE = re.compile(r"\x1b\[[0-9;]*[A-Za-z]")
SUMMARY_RE = re.compile(
    r"^\[(?P<config>[^\]]+)\]\s+\d+% tests passed,\s+(?P<failed>\d+) tests failed out of (?P<total>\d+)"
)
# Items of the "The following tests FAILED:" / "The following tests did not run:" lists at the end of a ctest run
LIST_ITEM_RE = re.compile(
    r"^\[(?P<config>[^\]]+)\]\s+(?P<num>\d+)\s*-\s*(?P<name>.+?)"
    r"(?:\s+\((?P<cause>[^)]+)\))?\s*$"
)
FAILED_LIST_START_RE = re.compile(r"^\[(?P<config>[^\]]+)\]\s+The following tests FAILED:\s*$")
NOT_RUN_LIST_START_RE = re.compile(r"^\[(?P<config>[^\]]+)\]\s+The following tests did not run:\s*$")
TEST_OUTPUT_RE = re.compile(
    r"^\[(?P<config>[^\]]+)\]\s+(?P<num>\d+):\s*(?P<text>.*)$"
)
START_RE = re.compile(
    #         Start 229: py_camera_max_resolution
    r"^\[(?P<config>[^\]]+)\]\s+Start\s+(?P<num>\d+):\s+(?P<name>.+?)\s*$"
)
RESULT_RE = re.compile(
    # matches the per-test result line of every outcome, e.g.:
    # 103/172 Test #185: gpu_stereo ...............................   Passed   58.15 sec
    #   1/178 Test  #37: camera_stream_restart_test ...............***Failed   29.20 sec
    #  51/172 Test #107: april_tags ...............................***Skipped  33.29 sec
    #  27/177 Test  #90: fsync_test ...............................***Timeout 1000.08 sec
    r"^\[(?P<config>[^\]]+)\]\s+\d+/(?P<planned>\d+)\s+Test\s+#(?P<num>\d+):\s+(?P<name>.+?)\s+\.*\s*"
    r"(?P<status>Passed|\*\*\*.+?)\s+(?P<seconds>\d+(?:\.\d+)?)\s+sec\s*$"
)
TESTSUITE_RE = re.compile(
    # print(f"Running tests for configuration: linux_rvc2_test / RVC2 - POE, on platform: RVC4, on protocol: POE, with labels: {labels if labels is not None else ""}")
    r"^(Running tests for configuration: )(?P<name>.+?), on platform: (?P<platform>.+?), on protocol: (?P<protocol>.+?), with labels:\s*(?P<labels>.*)$"
)
TESTBED_RE = re.compile(
    # [INFO] Using testbed: slo304-4d
    r"^\d{4}-\d{2}-\d{2}\s+\d{2}:\d{2}:\d{2},\d{3}\s+\[INFO\]\s+Using\s+testbed:\s+(?P<name>.+?)\s*$"
)

def normalize_name(name: str) -> str:
    return " ".join(name.strip().split())


def parse_args(argv):
    if len(argv) < 3:
        return None, None, ""

    log_path = Path(argv[1])
    junit_path = Path(argv[2])
    context = " ".join(argv[3:]).strip()
    return log_path, junit_path, context

def classify_status(status: str):
    """Map a ctest result status ("Passed", "***Failed", "***Timeout", ...) to (kind, cause)."""
    cause = normalize_name(status.lstrip("*"))
    if cause == "Passed":
        return "passed", ""
    if cause.startswith("Skipped") or cause == "Not Run (Disabled)":
        return "skipped", cause
    return "failed", cause


def parse_log(log_path: Path):
    order = []
    summaries = {}
    failures = {}
    passes = {}
    skips = {}
    test_outputs = {}
    descriptions = {}
    started = {}
    planned = {}
    testbedId = None
    # Config whose "The following tests FAILED:" / "did not run:" list is being read, and which list it is
    list_config = None
    list_kind = None

    def ensure_config(config: str) -> None:
        if config not in summaries:
            summaries[config] = None
            failures[config] = OrderedDict()
            passes[config] = OrderedDict()
            skips[config] = OrderedDict()
            test_outputs[config] = {}
            started[config] = OrderedDict()
            order.append(config)
            descriptions[config] = {"name": "null", "DEPTHAI_PLATFORM": 'null', "DEPTHAI_PROTOCOL": 'null', "labels": ''}

    def add_failure(config: str, num: str, name: str, cause: str) -> None:
        if num not in failures[config]:
            failures[config][num] = (name, cause)
        else:
            _, prev_cause = failures[config][num]
            if not prev_cause and cause:
                failures[config][num] = (name, cause)

    with log_path.open("r", errors="ignore") as handle:
        for raw_line in handle:
            line = ANSI_RE.sub("", raw_line.rstrip())
            if not line:
                continue

            summary_match = SUMMARY_RE.search(line)
            if summary_match:
                config = summary_match.group("config").strip()
                ensure_config(config)
                failed = int(summary_match.group("failed"))
                total = int(summary_match.group("total"))
                passed = total - failed
                summaries[config] = (passed, failed, total)

            description_match = TESTSUITE_RE.search(line)
            if description_match:
                config = description_match.group("name").strip()
                ensure_config(config)
                descriptions[config]["name"] = config
                descriptions[config]["DEPTHAI_PLATFORM"] = description_match.group("platform").strip()
                descriptions[config]["DEPTHAI_PROTOCOL"] = description_match.group("protocol").strip()
                descriptions[config]["labels"] = description_match.group("labels").strip()

            output_match = TEST_OUTPUT_RE.search(line)
            if output_match:
                config = output_match.group("config").strip()
                ensure_config(config)
                num = output_match.group("num").strip()
                text = output_match.group("text").rstrip()
                if text:
                    test_outputs[config].setdefault(num, []).append(text)

            start_match = START_RE.search(line)
            if start_match:
                config = start_match.group("config").strip()
                ensure_config(config)
                started[config][start_match.group("num")] = normalize_name(start_match.group("name"))

            # Every test gets a result line, also when the run is cancelled before the final summary
            result_match = RESULT_RE.search(line)
            if result_match:
                config = result_match.group("config").strip()
                ensure_config(config)
                num = result_match.group("num").strip()
                name = normalize_name(result_match.group("name"))
                seconds = float(result_match.group("seconds"))
                planned[config] = max(planned.get(config, 0), int(result_match.group("planned")))
                kind, cause = classify_status(result_match.group("status"))
                if kind == "passed":
                    passes[config][num] = (name, seconds)
                elif kind == "skipped":
                    skips[config][num] = (name, seconds)
                else:
                    add_failure(config, num, name, cause)

            list_start = FAILED_LIST_START_RE.search(line) or NOT_RUN_LIST_START_RE.search(line)
            if list_start:
                list_config = list_start.group("config").strip()
                list_kind = "failed" if "FAILED" in line else "skipped"
                ensure_config(list_config)
                continue

            # Only accept list items directly below a list header, other "[cfg] N - text" lines are test output
            item_match = LIST_ITEM_RE.search(line) if list_config is not None else None
            if item_match and item_match.group("config").strip() == list_config:
                num = item_match.group("num").strip()
                name = normalize_name(item_match.group("name"))
                cause = item_match.group("cause")
                cause = normalize_name(cause) if cause else ""
                if list_kind == "failed":
                    add_failure(list_config, num, name, cause)
                elif num not in skips[list_config]:
                    skips[list_config][num] = (name, 0.0)
            elif list_config is not None and line.startswith(f"[{list_config}]"):
                list_config = None
                list_kind = None

            if testbedId is None:
                testbedMatch = TESTBED_RE.search(line)
                if testbedMatch:
                    testbedId = testbedMatch.group("name")

    # Tests that started but have no result line: the run was cancelled or the log is truncated
    unfinished = {}
    for config in order:
        unfinished[config] = OrderedDict(
            (num, name)
            for num, name in started[config].items()
            if num not in passes[config] and num not in skips[config] and num not in failures[config]
        )
        for num, name in unfinished[config].items():
            add_failure(config, num, name, "Did not finish")
            test_outputs[config].setdefault(num, []).append("Test did not finish: no ctest result line (run cancelled or log truncated).")

    return order, summaries, failures, passes, skips, test_outputs, descriptions, testbedId, unfinished, planned


def iter_configs(order, summaries, failures):
    configs = list(order)
    for config in summaries:
        if config not in configs:
            configs.append(config)
    for config in failures:
        if config not in configs:
            configs.append(config)
    return configs


def build_failure_text(cause: str, output_lines):
    details = []
    if cause:
        details.append(f"Cause: {cause}")
    clipped_lines = [line for line in output_lines if line.strip()]
    if clipped_lines:
        details.append("Relevant test output:")
        details.extend(clipped_lines)
    else:
        details.append("Relevant test output: not captured.")

    return "\n".join(details)


def write_junit(
    junit_path: Path,
    context: str,
    order,
    summaries,
    failures,
    passes,
    skips,
    test_outputs,
    descriptions,
    testbedId,
    unfinished=None,
    planned=None
):
    unfinished = unfinished or {}
    planned = planned or {}
    root = ET.Element("testsuites")
    all_tests = 0
    all_failures = 0

    configs = iter_configs(order, summaries, failures)
    if not configs:
        empty_suite = ET.SubElement(
            root,
            "testsuite",
            name=f"{context or 'ctest'}",
            tests="1",
            failures="0",
            errors="0",
            skipped="0",
            time="0",
        )
        empty_case = ET.SubElement(
            empty_suite,
            "testcase",
            classname=f"{context or 'ctest'}",
            name="summary empty",
            time="0",
        )
        system_out = ET.SubElement(empty_case, "system-out")
        system_out.text = "No CTest summary lines found in the log."
        all_tests = 1

    for config in configs:
        summary = summaries.get(config)
        parsed_failures = failures.get(config, OrderedDict())
        parsed_passes = passes.get(config, OrderedDict())
        parsed_skips = skips.get(config, OrderedDict())
        description = descriptions.get(config, {"name": "null", "DEPTHAI_PLATFORM": 'null', "DEPTHAI_PROTOCOL": 'null', "labels": ''})
        parsed_unfinished = unfinished.get(config, OrderedDict())
        finished_count = len(parsed_passes) + len(parsed_skips) + len(parsed_failures) - len(parsed_unfinished)

        # An announced configuration can crash before any tests start.
        incomplete = summary is None and (
            description["name"] != "null" or finished_count > 0 or len(parsed_unfinished) > 0
        )
        written_failures = len(parsed_failures) + (1 if incomplete else 0)

        # Counts come from the per-test result lines; ctest's own "passed" count includes skipped tests
        suite_failures = max(summary[1] if summary else 0, written_failures)
        suite_passes = len(parsed_passes)
        suite_skips = len(parsed_skips)
        suite_name = f"{context} / {config}" if context else config
        suite_tests = suite_failures+suite_passes+suite_skips

        declared_failed = summary[1] if summary else suite_failures
        declared_total = summary[2] if summary else suite_tests
        declared_passed = summary[0] if summary else declared_total - declared_failed

        suite = ET.SubElement(
            root,
            "testsuite",
            name=suite_name,
            tests=str(suite_tests),
            failures=str(suite_failures),
            errors="?",
            skipped=str(suite_skips),
            time="0",
            labels=description['labels'],
            DEPTHAI_PLATFORM=description['DEPTHAI_PLATFORM'],
            DEPTHAI_PROTOCOL=description['DEPTHAI_PROTOCOL']
        )

        props = ET.SubElement(suite, "properties")
        ET.SubElement(
            props,
            "property",
            name="ctest.summary",
            value=f"Passed={declared_passed}, Failed={declared_failed}, Total={declared_total}",
        )

        for num, entry in parsed_failures.items():
            test_name, cause = entry
            output_lines = test_outputs.get(config, {}).get(num, [])
            case = ET.SubElement(
                suite,
                "testcase",
                classname=suite_name,
                name=f"#{num} {test_name}",
                time="0",
            )
            failure = ET.SubElement(
                case,
                "failure",
                type="failure",
                message=cause or "Failed",
            )
            failure.text = build_failure_text(
                cause=cause,
                output_lines=output_lines,
            )

        if incomplete:
            not_started = max(planned.get(config, 0) - finished_count - len(parsed_unfinished), 0)
            case = ET.SubElement(
                suite,
                "testcase",
                classname=suite_name,
                name="#0 ctest_incomplete",
                time="0",
            )
            failure = ET.SubElement(
                case,
                "failure",
                type="failure",
                message="CTest run did not complete",
            )
            failure.text = (
                "No CTest summary line found: the run was cancelled, timed out or crashed. "
                f"{finished_count} tests finished, {len(parsed_unfinished)} did not finish, "
                f"{not_started} of {planned.get(config, 0)} planned tests did not start."
            )

        missing_failures = suite_failures - written_failures
        for index in range(1, missing_failures + 1):
            case = ET.SubElement(
                suite,
                "testcase",
                classname=suite_name,
                name=f"unknown_failure {index}",
                time="0",
            )
            failure = ET.SubElement(
                case,
                "failure",
                type="failure",
                message="Unknown failure",
            )
            failure.text = "CTest reported an extra failure that was not listed by name."

        for num, entry in parsed_skips.items():
            test_name, time = entry
            case = ET.SubElement(
                suite,
                "testcase",
                classname=suite_name,
                name=f"#{num} {test_name}",
                time=str(time),
            )
            skip = ET.SubElement(
                case,
                "skip",
            )
        

        for num, entry in parsed_passes.items():
            test_name, time = entry
            case = ET.SubElement(
                suite,
                "testcase",
                classname=suite_name,
                name=f"#{num} {test_name}",
                time=str(time),
            )
            success = ET.SubElement(
                case,
                "success",
            )

        all_tests += suite_tests
        all_failures += suite_failures

    root.set("tests", str(all_tests))
    root.set("failures", str(all_failures))
    root.set("errors", "0")
    root.set("time", "0")
    root.set("testbedId", testbedId if testbedId is not None else "")

    tree = ET.ElementTree(root)
    if hasattr(ET, "indent"):
        ET.indent(tree, space="  ")
    junit_path.parent.mkdir(parents=True, exist_ok=True)
    tree.write(junit_path, encoding="utf-8", xml_declaration=True)


def main() -> int:
    log_path, junit_path, context = parse_args(sys.argv)
    if log_path is None or junit_path is None:
        print("Usage: ctest_to_junit.py <log_path> <junit_output> [context]")
        return 0

    if not log_path.exists():
        print(f"Log file not found: {log_path}")
        return 0

    order, summaries, failures, passes, skips, test_outputs, descriptions, testbedId, unfinished, planned = parse_log(log_path)
    write_junit(
        junit_path=junit_path,
        context=context,
        order=order,
        summaries=summaries,
        failures=failures,
        passes=passes,
        skips=skips,
        test_outputs=test_outputs,
        descriptions=descriptions,
        testbedId=testbedId,
        unfinished=unfinished,
        planned=planned
    )
    print(f"Wrote JUnit report: {junit_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
