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
        return None, "", "", "", "", 0

    log_path = Path(argv[1])
    context_parts = []
    log_url = ""
    raw_log_url = ""
    log_map_path = ""
    step_number = 0

    i = 2
    while i < len(argv):
        arg = argv[i]
        if arg == "--log-url":
            i += 1
            if i < len(argv):
                log_url = argv[i]
        elif arg.startswith("--log-url="):
            log_url = arg.split("=", 1)[1]
        elif arg == "--raw-log-url":
            i += 1
            if i < len(argv):
                raw_log_url = argv[i]
        elif arg.startswith("--raw-log-url="):
            raw_log_url = arg.split("=", 1)[1]
        elif arg == "--log-map":
            i += 1
            if i < len(argv):
                log_map_path = argv[i]
        elif arg.startswith("--log-map="):
            log_map_path = arg.split("=", 1)[1]
        elif arg == "--step-number":
            i += 1
            if i < len(argv):
                try:
                    step_number = int(argv[i])
                except ValueError:
                    step_number = 0
        elif arg.startswith("--step-number="):
            try:
                step_number = int(arg.split("=", 1)[1])
            except ValueError:
                step_number = 0
        else:
            context_parts.append(arg)
        i += 1

    context = " ".join(context_parts).strip()
    return log_path, context, log_url, raw_log_url, log_map_path, step_number


def load_log_info(log_map_path: str, context: str):
    if not log_map_path or not context:
        return "", "", 0
    try:
        with open(log_map_path, "r", encoding="utf-8") as handle:
            mapping = json.load(handle)
    except (OSError, json.JSONDecodeError):
        return "", "", 0

    artifact_name = f"ctest-log-{context}"
    entry = mapping.get(artifact_name, mapping.get(context, ""))
    if isinstance(entry, str):
        return entry, "", 0
    if not isinstance(entry, dict):
        return "", "", 0

    job_url = entry.get("job_url", entry.get("log_url", ""))
    raw_log_url = entry.get("raw_log_url", entry.get("raw_url", ""))
    step_number = entry.get("step_number", entry.get("step", 0))
    try:
        step_number = int(step_number)
    except (TypeError, ValueError):
        step_number = 0
    return job_url, raw_log_url, step_number


def main() -> int:
    log_path, context, log_url, raw_log_url, log_map_path, step_number = parse_args(
        sys.argv
    )
    if log_path is None:
        print("No log path provided.")
        return 0

    if not log_url and not raw_log_url and not step_number:
        log_url, raw_log_url, step_number = load_log_info(log_map_path, context)

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
            if log_url and step_number and fail_line:
                extras.append(f"[log]({log_url}#step:{step_number}:{fail_line})")
            elif log_url:
                extras.append(f"[log]({log_url})")
            if raw_log_url:
                extras.append(f"[raw logs]({raw_log_url})")
            extra_suffix = f" ({', '.join(extras)})" if extras else ""
            if cause:
                print(f"- [{config}] #{num} {name} - {cause}{extra_suffix}")
            else:
                print(f"- [{config}] #{num} {name}{extra_suffix}")
            any_failed = True
    if not any_failed:
        print("No tests failed.")
    print()

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
