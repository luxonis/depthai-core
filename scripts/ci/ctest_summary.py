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
    r"\[(?P<config>[^\]]+)\]\s+(?P<num>\d+)\s*-\s*(?P<name>.+?)\s*\(Failed\)"
)
FAILED_LINE_RE = re.compile(
    r"\[(?P<config>[^\]]+)\]\s+\d+/\d+\s+Test\s+#(?P<num>\d+):\s+(?P<name>.+?)\s+\.{3,}\*{3}Failed"
)


def normalize_name(name: str) -> str:
    return " ".join(name.strip().split())


def main() -> int:
    if len(sys.argv) < 2:
        print("No log path provided.")
        return 0

    log_path = Path(sys.argv[1])
    context = " ".join(sys.argv[2:]).strip()
    header_suffix = f" ({context})" if context else ""
    if not log_path.exists():
        print(f"Log file not found: {log_path}")
        return 0

    order = []
    summaries = {}
    failures = {}

    def ensure_config(config: str) -> None:
        if config not in summaries:
            summaries[config] = None
            failures[config] = OrderedDict()
            order.append(config)

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

            failed_match = FAILED_LIST_RE.search(line)
            if not failed_match:
                failed_match = FAILED_LINE_RE.search(line)
            if failed_match:
                config = failed_match.group("config").strip()
                ensure_config(config)
                num = failed_match.group("num").strip()
                name = normalize_name(failed_match.group("name"))
                failures[config].setdefault(num, name)

    print(f"## Test Summary{header_suffix}")
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

    print(f"\n## Failed Tests{header_suffix}")
    any_failed = False
    for config in order:
        config_failures = failures.get(config, {})
        for num, name in config_failures.items():
            print(f"- [{config}] #{num} {name}")
            any_failed = True
    if not any_failed:
        print("No failed tests found.")

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
