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
    test_outputs = {}

    def ensure_config(config: str) -> None:
        if config not in summaries:
            summaries[config] = None
            failures[config] = OrderedDict()
            test_outputs[config] = {}
            order.append(config)

    line_no = 0
    with log_path.open("r", errors="ignore") as handle:
        for raw_line in handle:
            line_no += 1
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
                    failures[config][num] = (name, cause, line_no)
                else:
                    prev_name, prev_cause, prev_line = failures[config][num]
                    if not prev_cause and cause:
                        failures[config][num] = (name, cause, line_no)
                    else:
                        failures[config][num] = (prev_name, prev_cause, prev_line)

    return order, summaries, failures, test_outputs


def iter_configs(order, summaries, failures):
    configs = list(order)
    for config in summaries:
        if config not in configs:
            configs.append(config)
    for config in failures:
        if config not in configs:
            configs.append(config)
    return configs


def clip_output_lines(lines):
    clipped = [line for line in lines if line.strip()]
    if not clipped:
        return [], False, False
    return clipped, False, False


def build_failure_text(cause: str, line_no: int, output_lines):
    details = []
    if cause:
        details.append(f"Cause: {cause}")
    if line_no:
        details.append(f"Failed list line: {line_no}")

    clipped_lines, _, _ = clip_output_lines(output_lines)
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
    test_outputs,
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

        declared_failed = summary[1] if summary else len(parsed_failures)
        declared_total = summary[2] if summary else len(parsed_failures)
        declared_passed = summary[0] if summary else max(declared_total - declared_failed, 0)

        suite_failures = max(declared_failed, len(parsed_failures))
        suite_name = f"{context} / {config}" if context else config
        suite_tests = max(declared_total, suite_failures)

        suite = ET.SubElement(
            root,
            "testsuite",
            name=suite_name,
            tests=str(suite_tests),
            failures=str(suite_failures),
            errors="0",
            skipped="0",
            time="0",
        )

        props = ET.SubElement(suite, "properties")
        ET.SubElement(
            props,
            "property",
            name="ctest.summary",
            value=f"Passed={declared_passed}, Failed={declared_failed}, Total={declared_total}",
        )

        for num, entry in parsed_failures.items():
            test_name, cause, fail_line = entry
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
                line_no=fail_line,
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

    order, summaries, failures, test_outputs = parse_log(log_path)
    write_junit(
        junit_path=junit_path,
        context=context,
        order=order,
        summaries=summaries,
        failures=failures,
        test_outputs=test_outputs,
    )
    print(f"Wrote JUnit report: {junit_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
