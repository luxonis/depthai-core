#!/usr/bin/env python3
import re
import sys
from collections import OrderedDict
from pathlib import Path


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

MAX_SNIPPET_LINES = 30
MAX_SNIPPET_CHARS = 20000


def normalize_name(name: str) -> str:
    return " ".join(name.strip().split())


def parse_args(argv):
    if len(argv) < 2:
        return None, ""

    log_path = Path(argv[1])
    context = " ".join(argv[2:]).strip()
    return log_path, context


def clip_output_lines(lines):
    clipped = [line for line in lines if line.strip()]
    if not clipped:
        return []
    if len(clipped) > MAX_SNIPPET_LINES:
        clipped = clipped[-MAX_SNIPPET_LINES:]

    selected_reversed = []
    chars = 0
    for line in reversed(clipped):
        separator = 1 if selected_reversed else 0
        needed = len(line) + separator
        if chars + needed > MAX_SNIPPET_CHARS:
            if not selected_reversed and MAX_SNIPPET_CHARS > 0:
                # Keep tail of very long single lines.
                fragment = line[-MAX_SNIPPET_CHARS:]
                if fragment:
                    selected_reversed.append(fragment)
            break
        selected_reversed.append(line)
        chars += needed
    return list(reversed(selected_reversed))


def main() -> int:
    log_path, context = parse_args(sys.argv)
    if log_path is None:
        print("No log path provided.")
        return 0

    header_suffix = f" ({context})" if context else ""
    if not log_path.exists():
        print(f"Log file not found: {log_path}")
        return 0

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

    print(f"## Test Summary {header_suffix}:")
    if not summaries:
        print("No test summary lines found in the log.")
    else:
        print("| Configuration | Passed | Failed | Total |")
        print("| --- | ---: | ---: | ---: |")
        for config in order:
            summary = summaries.get(config)
            if summary is None:
                print(f"| {config} | n/a | n/a | n/a |")
            else:
                passed, failed, total = summary
                print(f"| {config} | {passed} | {failed} | {total} |")

    print(f"\n## Failed Tests:")
    any_failed = False
    for config in order:
        config_failures = failures.get(config, {})
        for num, entry in config_failures.items():
            name, cause, _ = entry
            if cause:
                print(f"- [{config}] #{num} {name} - {cause}")
            else:
                print(f"- [{config}] #{num} {name}")

            snippet = clip_output_lines(test_outputs.get(config, {}).get(num, []))
            if snippet:
                print("```text")
                for snippet_line in snippet:
                    print(snippet_line)
                print("```")
            any_failed = True
    if not any_failed:
        print("No tests failed.")
    print()

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
