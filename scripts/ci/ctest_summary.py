#!/usr/bin/env python3
import json
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


def normalize_name(name: str) -> str:
    return " ".join(name.strip().split())


def parse_args(argv):
    if len(argv) < 2:
        return None, "", "", ""

    log_path = Path(argv[1])
    context_parts = []
    log_url = ""
    log_map_path = ""

    i = 2
    while i < len(argv):
        arg = argv[i]
        if arg == "--log-url":
            i += 1
            if i < len(argv):
                log_url = argv[i]
        elif arg.startswith("--log-url="):
            log_url = arg.split("=", 1)[1]
        elif arg == "--log-map":
            i += 1
            if i < len(argv):
                log_map_path = argv[i]
        elif arg.startswith("--log-map="):
            log_map_path = arg.split("=", 1)[1]
        else:
            context_parts.append(arg)
        i += 1

    context = " ".join(context_parts).strip()
    return log_path, context, log_url, log_map_path


def load_log_url(log_map_path: str, context: str) -> str:
    if not log_map_path or not context:
        return ""
    try:
        with open(log_map_path, "r", encoding="utf-8") as handle:
            mapping = json.load(handle)
    except (OSError, json.JSONDecodeError):
        return ""

    artifact_name = f"ctest-log-{context}"
    return mapping.get(artifact_name, mapping.get(context, ""))


def main() -> int:
    log_path, context, log_url, log_map_path = parse_args(sys.argv)
    if log_path is None:
        print("No log path provided.")
        return 0

    if not log_url:
        log_url = load_log_url(log_map_path, context)

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
            name, cause, fail_line = entry
            extras = []
            if fail_line:
                extras.append(f"line {fail_line}")
            if log_url:
                extras.append(f"[log]({log_url})")
            extra_suffix = f" ({', '.join(extras)})" if extras else ""
            if cause:
                print(f"- [{config}] #{num} {name} - {cause}{extra_suffix}")
            else:
                print(f"- [{config}] #{num} {name}{extra_suffix}")
            any_failed = True
    if not any_failed:
        print("No tests failed.")

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
