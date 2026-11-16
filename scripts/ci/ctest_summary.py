#!/usr/bin/env python3
import re
import sys
from collections import OrderedDict
from pathlib import Path


ANSI_RE = re.compile(r"\x1b\[[0-9;]*[A-Za-z]")
SUMMARY_RE = re.compile(
    r"^\[(?P<config>[^\]]+)\]\s+\d+% tests passed,\s+(?P<failed>\d+) tests failed out of (?P<total>\d+)"
)
# Items of the "The following tests FAILED:" list at the end of a ctest run
FAILED_LIST_RE = re.compile(
    r"^\[(?P<config>[^\]]+)\]\s+(?P<num>\d+)\s*-\s*(?P<name>.+?)"
    r"(?:\s+\((?P<cause>[^)]+)\))?\s*$"
)
FAILED_LIST_START_RE = re.compile(r"^\[(?P<config>[^\]]+)\]\s+The following tests FAILED:\s*$")
TEST_OUTPUT_RE = re.compile(
    r"^\[(?P<config>[^\]]+)\]\s+(?P<num>\d+):\s*(?P<text>.*)$"
)
START_RE = re.compile(r"^\[(?P<config>[^\]]+)\]\s+Start\s+(?P<num>\d+):\s+(?P<name>.+?)\s*$")
# Per-test result line, e.g. "  1/178 Test  #37: camera_stream_restart_test ...............***Failed   29.20 sec"
RESULT_RE = re.compile(
    r"^\[(?P<config>[^\]]+)\]\s+\d+/\d+\s+Test\s+#(?P<num>\d+):\s+(?P<name>.+?)\s+\.*\s*"
    r"(?P<status>Passed|\*\*\*.+?)\s+(?P<seconds>\d+(?:\.\d+)?)\s+sec\s*$"
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
    results = {}
    started = {}
    list_config = None

    def ensure_config(config: str) -> None:
        if config not in summaries:
            summaries[config] = None
            failures[config] = OrderedDict()
            test_outputs[config] = {}
            results[config] = {}
            started[config] = OrderedDict()
            order.append(config)

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
                status = normalize_name(result_match.group("status").lstrip("*"))
                if status == "Passed":
                    results[config][num] = "passed"
                elif status.startswith("Skipped") or status == "Not Run (Disabled)":
                    results[config][num] = "skipped"
                else:
                    results[config][num] = "failed"
                    add_failure(config, num, normalize_name(result_match.group("name")), status)

            list_start = FAILED_LIST_START_RE.search(line)
            if list_start:
                list_config = list_start.group("config").strip()
                ensure_config(list_config)
                continue

            # Only accept list items directly below the list header, other "[cfg] N - text" lines are test output
            failed_match = FAILED_LIST_RE.search(line) if list_config is not None else None
            if failed_match and failed_match.group("config").strip() == list_config:
                num = failed_match.group("num").strip()
                name = normalize_name(failed_match.group("name"))
                cause = failed_match.group("cause")
                cause = normalize_name(cause) if cause else ""
                add_failure(list_config, num, name, cause)
            elif list_config is not None and line.startswith(f"[{list_config}]"):
                list_config = None

    # Tests that started but have no result line: the run was cancelled or the log is truncated
    for config in order:
        for num, name in started[config].items():
            if num not in results[config] and num not in failures[config]:
                add_failure(config, num, name, "Did not finish")

    print(f"## Test Summary{header_suffix}:")
    if not summaries:
        print("No test summary lines found in the log.")
    else:
        print("| Configuration | Passed | Failed | Total |")
        print("| --- | ---: | ---: | ---: |")
        for config in order:
            summary = summaries.get(config)
            if summary is None and not results[config] and not failures[config]:
                print(f"| {config} | n/a | n/a | n/a |")
            elif summary is None:
                # No ctest summary: count the per-test result lines (ctest counts skipped tests as passed)
                failed = len(failures[config])
                total = len(set(results[config]) | set(failures[config]))
                print(f"| {config} (incomplete, no ctest summary) | {total - failed} | {failed} | {total} |")
            else:
                passed, failed, total = summary
                print(f"| {config} | {passed} | {failed} | {total} |")

    print()
    print("<details>")
    print("<summary><strong>Failed Tests</strong></summary>")
    print()
    any_failed = False
    for config in order:
        config_failures = failures.get(config, {})
        for num, entry in config_failures.items():
            name, cause = entry
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
    print("</details>")
    print()

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
