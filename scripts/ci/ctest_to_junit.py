#!/usr/bin/env python3
import re
import sys
from collections import OrderedDict
from pathlib import Path
from xml.etree import ElementTree as ET


ANSI_RE = re.compile(r"\x1b\[[0-9;]*[A-Za-z]")
SUMMARY_RE = re.compile(
    r"\[(?P<config>[^\]]+)\]\s+\d+% tests passed,\s+(?P<failed>\d+) tests failed out of (?P<total>\d+)"
)
FAILED_LIST_RE = re.compile(
    r"\[(?P<config>[^\]]+)\]\s+(?P<num>\d+)\s*-\s*(?P<name>.+?)"
    r"(?:\s+\((?P<cause>[^)]+)\))?\s*$"
)
TEST_OUTPUT_RE = re.compile(
    r"\[(?P<config>[^\]]+)\]\s+(?P<num>\d+):\s*(?P<text>.*)$"
)
PASSED_RE = re.compile(
    # matches similar lines: 103/172 Test #185: gpu_stereo ...............................   Passed   58.15 sec
    r"^\[(?P<config>[^\]]+)\]\s+\d+/\d+\s+Test\s+#(?P<num>\d+):\s+(?P<name>.+?)\s+\.{3,}\s+Passed\s+(?P<seconds>\d+(?:\.\d+)?)\s+sec\s*$"
)
TESTSUITE_RE = re.compile(
    # print(f"Running tests for configuration: linux_rvc2_test / RVC2 - POE, on platform: RVC4, on protocol: POE, with labels: {labels if labels is not None else ""}")
    r"^(Running tests for configuration: )(?P<name>.+?), on platform: (?P<platform>.+?), on protocol: (?P<protocol>.+?), with labels: (?P<labels>.+?)$"
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

def parse_log(log_path: Path):
    order = []
    summaries = {}
    failures = {}
    passes = {}
    test_outputs = {}
    descriptions = {}

    def ensure_config(config: str) -> None:
        if config not in summaries:
            summaries[config] = None
            failures[config] = OrderedDict()
            passes[config] = OrderedDict()
            test_outputs[config] = {}
            order.append(config)
            descriptions[config] = {"name": "null", "DEPTHAI_PLATFORM": 'null', "DEPTHAI_PROTOCOL": 'null', "labels": ''}

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

            failed_match = FAILED_LIST_RE.search(line)
            if failed_match:
                config = failed_match.group("config").strip()
                ensure_config(config)
                num = failed_match.group("num").strip()
                name = normalize_name(failed_match.group("name"))
                cause = failed_match.group("cause")
                cause = normalize_name(cause) if cause else ""
                if num not in failures[config]:
                    failures[config][num] = (name, cause)
                else:
                    prev_name, prev_cause = failures[config][num]
                    if not prev_cause and cause:
                        failures[config][num] = (name, cause)
                    else:
                        failures[config][num] = (prev_name, prev_cause)

            passed_match = PASSED_RE.search(line)
            if passed_match:
                config = passed_match.group("config").strip()
                ensure_config(config)
                num = passed_match.group("num").strip()
                name = normalize_name(passed_match.group("name"))
                seconds = float(passed_match.group('seconds'))
                passes[config][num] = (name, seconds)

    return order, summaries, failures, passes, test_outputs, descriptions


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
    test_outputs,
    descriptions
):
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
            name="summary",
            time="0",
        )
        system_out = ET.SubElement(empty_case, "system-out")
        system_out.text = "No CTest summary lines found in the log."
        all_tests = 1

    for config in configs:
        summary = summaries.get(config)
        parsed_failures = failures.get(config, OrderedDict())
        parsed_passes = passes.get(config, OrderedDict())
        description = descriptions.get(config, {"name": "null", "DEPTHAI_PLATFORM": 'null', "DEPTHAI_PROTOCOL": 'null', "labels": ''})
        
        declared_failed = summary[1] if summary else len(parsed_failures)
        declared_total = summary[2] if summary else len(parsed_failures)
        declared_passed = summary[0] if summary else max(declared_total - declared_failed, 0)

        suite_failures = max(declared_failed, len(parsed_failures))
        suite_passes = max(declared_passed, len(parsed_passes))
        suite_name = f"{context} / {config}" if context else config
        suite_tests = suite_failures+suite_passes

        suite = ET.SubElement(
            root,
            "testsuite",
            name=suite_name,
            tests=str(suite_tests),
            failures=str(suite_failures),
            errors="0",
            skipped="0",
            time="0",
            labels=description['labels'],
            DEPTHAI_PLATFORM=descriptions['DEPTHAI_PLATFORM'],
            DEPTHAI_PROTOCOL=descriptions['DEPTHAI_PROTOCOL']
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

        missing_failures = suite_failures - len(parsed_failures)
        for index in range(1, missing_failures + 1):
            case = ET.SubElement(
                suite,
                "testcase",
                classname=suite_name,
                name=f"unknown failure {index}",
                time="0",
            )
            failure = ET.SubElement(
                case,
                "failure",
                type="failure",
                message="Unknown failure",
            )
            failure.text = "CTest reported an extra failure that was not listed by name."

        for num, entry in parsed_passes.items():
            test_name, time = entry
            case = ET.SubElement(
                suite,
                "testcase",
                classname=suite_name,
                name=f"#{num} {test_name}",
                time=str(time),
            )
            failure = ET.SubElement(
                case,
                "success",
            )

        all_tests += suite_tests
        all_failures += suite_failures

    root.set("tests", str(all_tests))
    root.set("failures", str(all_failures))
    root.set("errors", "0")
    root.set("time", "0")

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

    order, summaries, failures, passes, test_outputs, descriptions = parse_log(log_path)
    write_junit(
        junit_path=junit_path,
        context=context,
        order=order,
        summaries=summaries,
        failures=failures,
        passes=passes,
        test_outputs=test_outputs,
        descriptions=descriptions
    )
    print(f"Wrote JUnit report: {junit_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
