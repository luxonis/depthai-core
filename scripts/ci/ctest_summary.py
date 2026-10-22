#!/usr/bin/env python3
import sys
from pathlib import Path

from ctest_to_junit import parse_log


MAX_SNIPPET_LINES = 30
MAX_SNIPPET_CHARS = 20000


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

    order, summaries, failures, passes, skips, test_outputs, *_ = parse_log(log_path)

    print(f"## Test Summary{header_suffix}:")
    if not summaries:
        print("No test summary lines found in the log.")
    else:
        print("| Configuration | Passed | Failed | Skipped | Total |")
        print("| --- | ---: | ---: | ---: | ---: |")
        for config in order:
            summary = summaries.get(config)
            skipped = len(skips[config])
            if summary is None:
                passed, failed = len(passes[config]), len(failures[config])
                total = passed + failed + skipped
                config += " (incomplete, no ctest summary)"
            else:
                passed, failed, total = summary
                passed = max(0, passed - skipped)
            print(f"| {config} | {passed} | {failed} | {skipped} | {total} |")

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
