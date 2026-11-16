#!/usr/bin/env python3
"""Summarize one run of hil_aux_soak.yml.

Each pass job uploads a pass.json (os, row, pass and, when the pass was skipped, not_started)
and the hil_runner output in soak.log. This script reads every pass under PASSES_DIR, writes
one record per test per pass to --out and prints a Markdown summary for the job page.
"""
import argparse
import json
import re
import statistics
from collections import Counter, defaultdict
from dataclasses import dataclass, field
from pathlib import Path

from ctest_to_junit import ANSI_RE, RESULT_RE, parse_log

OUTCOMES = ("passed", "failed", "timeout", "skipped", "no_device", "not_run")
FAILING = ("failed", "timeout")
FAILURE_LINES = 40
# A pass whose skips exceed the median of its OS and row by more than this most likely lost its device.
MASS_SKIP_MARGIN = 10

NO_DEVICE_RE = re.compile(r"No device available|produced the following exit code: 125\b")
SANITY_RE = re.compile(r"\[INFO\] Checking sanity (?P<testbed>\S+)")
ERROR_RE = re.compile(r"\[(?:ERROR|CRITICAL)\]\s*(?P<message>.*)$")
EXCEPTION_RE = re.compile(r"^\w+(?:Exception|Error): ")
TEST_OUTPUT_RE = re.compile(r"^\[(?P<config>[^\]]+)\]\s+(?P<num>\d+):(?P<text>.*)$")
CATCH2_RULE = "-" * 79
CATCH2_FAILED_RE = re.compile(r"^\S.*:\d+: FAILED:$")


@dataclass
class TestResult:
    outcome: str
    seconds: float | None = None
    failure: str = ""


@dataclass
class Pass:
    os: str
    row: str
    number: int
    outcome: str = "infra"
    testbed: str | None = None
    testbeds_tried: list[str] = field(default_factory=list)
    sanity_checks: int = 0
    tests: dict[str, TestResult] = field(default_factory=dict)
    no_device_as_pass: int = 0
    mass_skip: bool = False
    hil_error: str = ""
    note: str = ""

    @property
    def key(self):
        return (self.os, self.row, self.number)

    @property
    def testbed_changes(self):
        return max(len(self.testbeds_tried) - 1, 0)

    def count(self, *outcomes):
        return sum(result.outcome in outcomes for result in self.tests.values())


@dataclass
class Run:
    night: str
    run_id: int
    run_attempt: int
    soak_ref: str
    depthai_ref: str
    depthai_sha: str
    device_pin: str

    @property
    def core_label(self):
        """3.12.1 for a release tag such as v3.12.1, aux for any other ref."""
        match = re.fullmatch(r"v(\d+\.\d+\.\d+)", self.depthai_ref)
        return match.group(1) if match else "aux"

    def pairing(self, os):
        return f"{self.core_label}+{os.split('+')[0]}"


def read_test_output(log_path):
    """Each test's output lines and ctest time, by (config, test number).

    parse_log strips the leading whitespace and the blank lines that delimit a Catch2 failure, and
    keeps no time for a failed test.
    """
    output, seconds = defaultdict(list), {}
    with log_path.open(errors="replace") as handle:
        for raw in handle:
            line = ANSI_RE.sub("", raw).rstrip()
            result = RESULT_RE.search(line)
            if result:
                seconds[(result["config"].strip(), result["num"])] = float(result["seconds"])
                continue
            match = TEST_OUTPUT_RE.match(line)
            if match:
                text = match["text"]
                output[(match["config"].strip(), match["num"])].append(text.removeprefix(" "))
    return output, seconds


def catch2_failures(lines):
    """Each Catch2 assertion failure, from its FAILED: line to the blank line after it, under its test case name."""
    rules = [index for index, line in enumerate(lines) if line == CATCH2_RULE]
    names = {close: [line.strip() for line in lines[start + 1:close]]
             for start, close in zip(rules, rules[1:])
             if 1 < close - start <= 4 and all(lines[start + 1:close])}
    excerpt, case, shown_case = [], None, None
    for index, line in enumerate(lines):
        if index in names:
            case = names[index]
        if not CATCH2_FAILED_RE.match(line):
            continue
        if case and case != shown_case:
            excerpt.append(" / ".join(case))
            shown_case = case
        end = index
        while end < len(lines) and lines[end].strip():
            end += 1
        excerpt.extend(lines[index:end])
    return excerpt


def failure_text(cause, output_lines):
    """The Catch2 failures of a test, or the last lines of its output when it has none."""
    excerpt = catch2_failures(output_lines)[:FAILURE_LINES]
    if not excerpt:
        excerpt = [line for line in output_lines if line.strip()][-FAILURE_LINES:]
    return "\n".join([f"Cause: {cause}", *excerpt])


def read_hil_events(log_path, soak_pass):
    """Sanity checks and the last hil_runner error, from the lines outside the ctest output."""
    pending_error = False
    with log_path.open(errors="replace") as handle:
        for raw in handle:
            line = ANSI_RE.sub("", raw).strip()
            if not line or line.startswith("["):
                continue
            if pending_error:
                soak_pass.hil_error = line
                pending_error = False
            sanity = SANITY_RE.search(line)
            if sanity:
                soak_pass.sanity_checks += 1
                if sanity["testbed"] not in soak_pass.testbeds_tried:
                    soak_pass.testbeds_tried.append(sanity["testbed"])
            error = ERROR_RE.search(line)
            if error:
                soak_pass.hil_error = error["message"]
                pending_error = not error["message"]
            elif EXCEPTION_RE.match(line):
                soak_pass.hil_error = line


def read_pass(pass_json):
    meta = json.loads(pass_json.read_text())
    soak_pass = Pass(os=meta["os"], row=meta["row"], number=int(meta["pass"]))
    if meta.get("not_started"):
        soak_pass.outcome = "not_started"
        soak_pass.note = meta["not_started"]
        return soak_pass
    log_path = pass_json.with_name("soak.log")
    if not log_path.exists():
        soak_pass.note = "no soak.log in the pass artifact"
        return soak_pass

    read_hil_events(log_path, soak_pass)
    order, summaries, failures, passes, skips, outputs, _, testbed, _, _ = parse_log(log_path)
    test_output, test_seconds = read_test_output(log_path)
    soak_pass.testbed = testbed
    for config in order:
        config_outputs = outputs.get(config, {})
        for num, (name, seconds) in passes[config].items():
            no_device = bool(NO_DEVICE_RE.search("\n".join(config_outputs.get(num, []))))
            soak_pass.no_device_as_pass += no_device
            soak_pass.tests[name] = TestResult("no_device" if no_device else "passed", seconds)
        for num, (name, seconds) in skips[config].items():
            no_device = bool(NO_DEVICE_RE.search("\n".join(config_outputs.get(num, []))))
            soak_pass.tests[name] = TestResult("no_device" if no_device else "skipped", seconds)
        for num, (name, cause) in failures[config].items():
            if cause == "Did not finish":
                soak_pass.tests[name] = TestResult("not_run")
                continue
            outcome = "timeout" if cause == "Timeout" else "failed"
            soak_pass.tests[name] = TestResult(outcome, test_seconds.get((config, num)),
                                               failure_text(cause, test_output.get((config, num), [])))

    if not order:
        soak_pass.note = "no ctest output"
    elif all(summaries[config] is not None for config in order):
        soak_pass.outcome = "completed"
    else:
        soak_pass.outcome = "partial"
    return soak_pass


def collect_passes(passes_dir, os_versions, rows, pass_count):
    """Every expected pass, plus any other pass found; a pass without an artifact is infra."""
    found = {}
    for pass_json in sorted(passes_dir.rglob("pass.json")):
        soak_pass = read_pass(pass_json)
        found[soak_pass.key] = soak_pass
    for os in os_versions:
        for row in rows:
            for number in range(1, pass_count + 1):
                if (os, row, number) not in found:
                    found[(os, row, number)] = Pass(os, row, number, note="no pass artifact")
    return sorted(found.values(), key=lambda p: (p.os, p.row, p.number))


def mark_mass_skips(passes):
    groups = defaultdict(list)
    for soak_pass in passes:
        if soak_pass.outcome in ("completed", "partial"):
            groups[(soak_pass.os, soak_pass.row)].append(soak_pass)
    for group in groups.values():
        median = statistics.median(p.count("skipped", "no_device") for p in group)
        for soak_pass in group:
            soak_pass.mass_skip = soak_pass.count("skipped", "no_device") > median + MASS_SKIP_MARGIN


def build_records(run, passes):
    """One record per test per pass. A test seen in any pass of the same OS and row is not_run where it is missing."""
    universe = defaultdict(set)
    for soak_pass in passes:
        universe[(soak_pass.os, soak_pass.row)].update(soak_pass.tests)
    records = []
    for soak_pass in passes:
        for test in sorted(universe[(soak_pass.os, soak_pass.row)]):
            result = soak_pass.tests.get(test, TestResult("not_run"))
            records.append({
                "night": run.night, "run_id": run.run_id, "run_attempt": run.run_attempt,
                "soak_ref": run.soak_ref, "depthai_ref": run.depthai_ref, "depthai_sha": run.depthai_sha,
                "device_pin": run.device_pin, "pairing": run.pairing(soak_pass.os), "os": soak_pass.os,
                "row": soak_pass.row, "pass": soak_pass.number, "pass_outcome": soak_pass.outcome,
                "testbed": soak_pass.testbed, "test": test, "outcome": result.outcome,
                "seconds": result.seconds, "failure": result.failure,
            })
    return records


def pass_summary(run, soak_pass):
    return {
        "pairing": run.pairing(soak_pass.os), "os": soak_pass.os, "row": soak_pass.row,
        "pass": soak_pass.number, "pass_outcome": soak_pass.outcome, "testbed": soak_pass.testbed,
        "testbeds_tried": soak_pass.testbeds_tried, "sanity_checks": soak_pass.sanity_checks,
        "testbed_changes": soak_pass.testbed_changes,
        "counts": {outcome: soak_pass.count(outcome) for outcome in OUTCOMES if outcome != "not_run"},
        "no_device_as_pass": soak_pass.no_device_as_pass, "mass_skip": soak_pass.mass_skip,
        "hil_error": soak_pass.hil_error, "note": soak_pass.note,
    }


def per_test(records):
    """Outcome counts and failing testbeds per (pairing, row, test)."""
    tests = defaultdict(lambda: {"outcomes": Counter(), "failed_on": [], "completed_failures": 0})
    for record in records:
        entry = tests[(record["pairing"], record["row"], record["test"])]
        entry["outcomes"][record["outcome"]] += 1
        if record["outcome"] in FAILING:
            entry["failed_on"].append(f"p{record['pass']} {record['testbed']}")
            entry["completed_failures"] += record["pass_outcome"] == "completed"
    return tests


def cell(value):
    return str(value).replace("|", "\\|").replace("\n", " ")


def table(header, rows):
    lines = ["| " + " | ".join(header) + " |", "|" + " --- |" * len(header)]
    lines += ["| " + " | ".join(cell(value) for value in row) + " |" for row in rows]
    return "\n".join(lines)


def hidden_effects(soak_pass):
    """The ways a pass can hide a failure: exit 125 counted as passed, mass skips, a testbed change."""
    effects = []
    if soak_pass.no_device_as_pass:
        effects.append(f"{soak_pass.no_device_as_pass} tests exited 125 and counted as passed")
    if soak_pass.mass_skip:
        effects.append(f"{soak_pass.count('skipped', 'no_device')} skipped "
                       f"({soak_pass.count('no_device')} without a device)")
    if soak_pass.testbed_changes:
        effects.append(f"testbed changed after a failed sanity check: {', '.join(soak_pass.testbeds_tried)}")
    return effects


def markdown(run, passes, records):
    completed = Counter((run.pairing(p.os), p.row) for p in passes if p.outcome == "completed")
    flaky, always = [], []
    for (pairing, row, test), entry in sorted(per_test(records).items()):
        outcomes = entry["outcomes"]
        if outcomes["passed"] and (outcomes["failed"] or outcomes["timeout"]):
            flaky.append((pairing, row, test, outcomes["passed"], outcomes["failed"], outcomes["timeout"],
                          ", ".join(entry["failed_on"])))
        if completed[(pairing, row)] and entry["completed_failures"] == completed[(pairing, row)]:
            always.append((pairing, row, test, entry["completed_failures"]))
    hidden, incomplete, per_pass = [], [], []
    for p in passes:
        where = (run.pairing(p.os), p.row, p.number)
        hidden += [(*where, p.testbed or "", effect) for effect in hidden_effects(p)]
        if p.outcome != "completed":
            incomplete.append((*where, p.outcome, p.testbed or "", p.note or p.hil_error))
        per_pass.append((*where, p.testbed or "", p.outcome,
                         *(p.count(o) for o in ("passed", "failed", "timeout", "skipped", "no_device")),
                         p.sanity_checks))

    out = [
        f"## Aux soak: DepthAI `{run.depthai_ref}` at `{run.depthai_sha[:12]}`, device `{run.device_pin}`",
        "",
        f"Night {run.night}, run {run.run_id}, attempt {run.run_attempt}.",
    ]
    if run.run_attempt > 1:
        out.append("**This is a re-run. A re-run replaces the failed jobs of the first attempt and hides their flakes.**")
    sections = [
        ("Flaky tests (passed and failed in this run)",
         ("Pairing", "Row", "Test", "Passed", "Failed", "Timeout", "Failed on"), flaky),
        ("Failed in every completed pass", ("Pairing", "Row", "Test", "Completed passes"), always),
        ("Hidden failures", ("Pairing", "Row", "Pass", "Testbed", "Effect"), hidden),
        ("Passes without a full result", ("Pairing", "Row", "Pass", "Outcome", "Testbed", "Note or last hil_runner error"),
         incomplete),
        ("Passes", ("Pairing", "Row", "Pass", "Testbed", "Outcome", "Passed", "Failed", "Timeout", "Skipped",
                    "No device", "Sanity checks"), per_pass),
    ]
    for title, header, rows in sections:
        out += ["", f"### {title}", ""]
        out.append(table(header, rows) if rows else "None.")
    return "\n".join(out)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("passes_dir", type=Path)
    parser.add_argument("--out", type=Path, required=True)
    parser.add_argument("--night", required=True)
    parser.add_argument("--run-id", type=int, required=True)
    parser.add_argument("--run-attempt", type=int, required=True)
    parser.add_argument("--soak-ref", required=True)
    parser.add_argument("--depthai-ref", required=True)
    parser.add_argument("--depthai-sha", default="")
    parser.add_argument("--device-pin", default="")
    parser.add_argument("--os-versions", type=json.loads, required=True, help="JSON list")
    parser.add_argument("--rows", type=json.loads, required=True, help="JSON list")
    parser.add_argument("--passes", type=int, required=True)
    args = parser.parse_args(argv)

    run = Run(args.night, args.run_id, args.run_attempt, args.soak_ref, args.depthai_ref, args.depthai_sha,
              args.device_pin)
    passes = collect_passes(args.passes_dir, args.os_versions, args.rows, args.passes)
    mark_mass_skips(passes)
    records = build_records(run, passes)
    results = {
        "run": vars(run) | {"core": run.core_label},
        "passes": [pass_summary(run, p) for p in passes],
        "records": records,
    }
    args.out.write_text(json.dumps(results, indent=1) + "\n")
    print(markdown(run, passes, records))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
