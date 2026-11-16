#!/usr/bin/env python3
"""Keep a history of HIL aux soak results and compare the DepthAI and OS pairings across nights.

  fetch    add the soak-results artifact of every completed hil_aux_soak.yml run that is not in the history
  ingest   add soak-results.json files to the history
  report   print the nights and, per test, the failure rate of each pairing and what the pattern points to

The history is a SQLite file. Ingesting a run again replaces it, so fetch and ingest can run any number of times.
"""
import argparse
import json
import sqlite3
import subprocess
import sys
import tempfile
from collections import Counter, defaultdict
from dataclasses import dataclass
from pathlib import Path

WORKFLOW = "hil_aux_soak.yml"
FAILING = ("failed", "timeout")
DECIDED = ("passed", "failed", "timeout")

# Keyed by whether the baseline, the aux DepthAI on the aux OS and the aux DepthAI on the old OS fail the test.
READINGS = {
    (True, True, True): "Older than both changes; compare with the develop nightly",
    (False, True, True): "The DepthAI change",
    (False, True, False): "Needs the aux DepthAI and the aux OS together",
    (True, True, False): "The OS change, independent of DepthAI",
    (True, False, False): "3.12.1 on the aux OS only",
    (False, False, True): "The aux DepthAI on an OS without the driver",
    (True, False, True): "No clear pattern",
    (False, False, False): "Below the failure threshold in every pairing",
}

SCHEMA = """
CREATE TABLE IF NOT EXISTS runs (
    run_id INTEGER PRIMARY KEY, run_attempt INTEGER, night TEXT, soak_ref TEXT,
    depthai_ref TEXT, depthai_sha TEXT, device_pin TEXT);
CREATE TABLE IF NOT EXISTS passes (
    run_id INTEGER, night TEXT, pairing TEXT, os TEXT, row TEXT, pass INTEGER, pass_outcome TEXT,
    testbed TEXT, testbed_changes INTEGER, no_device_as_pass INTEGER, mass_skip INTEGER);
CREATE TABLE IF NOT EXISTS results (
    run_id INTEGER, night TEXT, pairing TEXT, os TEXT, row TEXT, pass INTEGER, pass_outcome TEXT,
    testbed TEXT, test TEXT, outcome TEXT, seconds REAL, failure TEXT);
"""


def connect(path):
    con = sqlite3.connect(path)
    con.executescript(SCHEMA)
    return con


def ingest(con, results_path):
    data = json.loads(Path(results_path).read_text())
    run = data["run"]
    night = run["night"]
    with con:
        for table in ("runs", "passes", "results"):
            con.execute(f"DELETE FROM {table} WHERE run_id = ?", (run["run_id"],))
        con.execute(
            "INSERT INTO runs VALUES (?, ?, ?, ?, ?, ?, ?)",
            (run["run_id"], run["run_attempt"], night, run["soak_ref"], run["depthai_ref"], run["depthai_sha"],
             run["device_pin"]),
        )
        con.executemany(
            "INSERT INTO passes VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?)",
            [(run["run_id"], night, p["pairing"], p["os"], p["row"], p["pass"], p["pass_outcome"], p["testbed"],
              p["testbed_changes"], p["no_device_as_pass"], p["mass_skip"]) for p in data["passes"]],
        )
        con.executemany(
            "INSERT INTO results VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?)",
            [(run["run_id"], night, r["pairing"], r["os"], r["row"], r["pass"], r["pass_outcome"], r["testbed"],
              r["test"], r["outcome"], r["seconds"], r["failure"]) for r in data["records"]],
        )
    return run["run_id"], len(data["records"])


def gh(*args):
    return subprocess.run(["gh", *args], check=True, capture_output=True, text=True).stdout


def fetch(con, repo, limit):
    runs = json.loads(gh("run", "list", "--repo", repo, "--workflow", WORKFLOW, "--status", "completed",
                         "--limit", str(limit), "--json", "databaseId"))
    known = {run_id for (run_id,) in con.execute("SELECT run_id FROM runs")}
    for run_id in sorted(run["databaseId"] for run in runs if run["databaseId"] not in known):
        with tempfile.TemporaryDirectory() as tmp:
            try:
                gh("run", "download", str(run_id), "--repo", repo, "-n", "soak-results", "-D", tmp)
            except subprocess.CalledProcessError as error:
                print(f"run {run_id}: no soak-results artifact ({error.stderr.strip()})", file=sys.stderr)
                continue
            print("run %d: %d records" % ingest(con, Path(tmp) / "soak-results.json"))


def window(con, nights):
    """The last `nights` nights that have a soak run."""
    rows = con.execute("SELECT DISTINCT night FROM runs ORDER BY night DESC LIMIT ?", (nights,)).fetchall()
    return sorted(night for (night,) in rows)


@dataclass
class TestComparison:
    row: str
    test: str
    failures: dict  # pairing -> (failed or timed out, decided results)
    flaky_nights: int
    failure_testbeds: Counter
    reading: str


def compare(con, nights, pairings, min_failures):
    """Tests that failed at least once in the window, with the reading of their failure pattern."""
    marks = ",".join("?" * len(nights))
    stats = defaultdict(lambda: defaultdict(Counter))
    per_run = defaultdict(Counter)
    testbeds = defaultdict(Counter)
    query = f"SELECT run_id, night, pairing, row, test, outcome, testbed FROM results WHERE night IN ({marks})"
    for run_id, night, pairing, row, test, outcome, testbed in con.execute(query, nights):
        stats[(row, test)][pairing][outcome] += 1
        per_run[(row, test, run_id, pairing, night)][outcome] += 1
        if outcome in FAILING:
            testbeds[(row, test)][testbed] += 1

    flaky_nights = defaultdict(set)
    for (row, test, _, _, night), outcomes in per_run.items():
        if outcomes["passed"] and any(outcomes[o] for o in FAILING):
            flaky_nights[(row, test)].add(night)

    comparisons = []
    for (row, test), by_pairing in sorted(stats.items()):
        failures = {
            pairing: (sum(by_pairing[pairing][o] for o in FAILING), sum(by_pairing[pairing][o] for o in DECIDED))
            for pairing in pairings
        }
        if not any(failed for failed, _ in failures.values()):
            continue
        if any(decided == 0 for _, decided in failures.values()):
            reading = "Not run in every pairing"
        else:
            reading = READINGS[tuple(failures[pairing][0] >= min_failures for pairing in pairings)]
        comparisons.append(TestComparison(row, test, failures, len(flaky_nights[(row, test)]), testbeds[(row, test)],
                                          reading))
    return comparisons


def night_trend(con, nights):
    """Per night and pairing: passes, failures, flaky tests and the failure-hiding effects."""
    marks = ",".join("?" * len(nights))
    trend = {}
    for night, pairing, run_id, passes, completed, no_device, mass_skip, changes in con.execute(
        f"""SELECT night, pairing, run_id, COUNT(*), SUM(pass_outcome = 'completed'), SUM(no_device_as_pass),
                   SUM(mass_skip), SUM(testbed_changes > 0)
            FROM passes WHERE night IN ({marks}) GROUP BY night, pairing, run_id""",
        nights,
    ):
        trend[(night, pairing, run_id)] = {"passes": f"{completed}/{passes}", "hidden": f"{no_device}/{mass_skip}/{changes}"}
    for night, pairing, run_id, failures, failing_tests in con.execute(
        f"""SELECT night, pairing, run_id, SUM(outcome IN ('failed', 'timeout')),
                   COUNT(DISTINCT CASE WHEN outcome IN ('failed', 'timeout') THEN row || ' ' || test END)
            FROM results WHERE night IN ({marks}) GROUP BY night, pairing, run_id""",
        nights,
    ):
        trend[(night, pairing, run_id)] |= {"failures": failures, "failing_tests": failing_tests}
    for night, pairing, run_id, flaky in con.execute(
        f"""SELECT night, pairing, run_id, COUNT(*) FROM (
                SELECT night, pairing, run_id FROM results WHERE night IN ({marks})
                GROUP BY night, pairing, run_id, os, row, test
                HAVING SUM(outcome = 'passed') > 0 AND SUM(outcome IN ('failed', 'timeout')) > 0)
            GROUP BY night, pairing, run_id""",
        nights,
    ):
        trend[(night, pairing, run_id)]["flaky"] = flaky
    return trend


def table(header, rows):
    lines = ["| " + " | ".join(header) + " |", "|" + " --- |" * len(header)]
    lines += ["| " + " | ".join(str(value).replace("|", "\\|") for value in row) + " |" for row in rows]
    return "\n".join(lines)


def report(con, nights_back, pairings, min_failures):
    nights = window(con, nights_back)
    if not nights:
        return "No soak runs in the history."
    trend = night_trend(con, nights)
    out = [
        f"# Aux soak, {nights[0]} to {nights[-1]} ({len(nights)} nights)",
        "",
        "## Nights",
        "",
        "Hidden: tests that exited 125 and passed / passes with mass skips / passes that changed testbed.",
        "",
        table(
            ("Night", "Pairing", "Run", "Completed passes", "Failures", "Failing tests", "Flaky tests", "Hidden"),
            [(night, pairing, run_id, t["passes"], t.get("failures", 0), t.get("failing_tests", 0), t.get("flaky", 0),
              t["hidden"]) for (night, pairing, run_id), t in sorted(trend.items())],
        ),
        "",
        "## Tests that failed",
        "",
        f"Each pairing cell is failed or timed out / decided results. A pairing fails a test at {min_failures} or "
        "more failures. Compare only tests that exist in every pairing.",
        "",
    ]
    rows = [
        (c.row, c.test, *(f"{c.failures[p][0]}/{c.failures[p][1]}" for p in pairings), c.flaky_nights,
         ", ".join(f"{testbed} x{count}" for testbed, count in c.failure_testbeds.most_common()), c.reading)
        for c in compare(con, nights, pairings, min_failures)
    ]
    out.append(table(("Row", "Test", *pairings, "Flaky nights", "Failure testbeds", "Reading"), rows)
               if rows else "No test failed.")
    return "\n".join(out)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--db", default=str(Path.home() / ".local/state/aux-soak/history.sqlite"))
    commands = parser.add_subparsers(dest="command", required=True)
    fetch_parser = commands.add_parser("fetch")
    fetch_parser.add_argument("--repo", default="luxonis/depthai-core")
    fetch_parser.add_argument("--limit", type=int, default=60, help="newest soak runs to look at")
    ingest_parser = commands.add_parser("ingest")
    ingest_parser.add_argument("files", nargs="+", type=Path)
    report_parser = commands.add_parser("report")
    report_parser.add_argument("--nights", type=int, default=14)
    report_parser.add_argument("--baseline", default="3.12.1+aux-con-dev-4")
    report_parser.add_argument("--aux-new-os", default="aux+aux-con-dev-4")
    report_parser.add_argument("--aux-old-os", default="aux+1.44.0")
    report_parser.add_argument("--min-failures", type=int, default=2)
    args = parser.parse_args(argv)

    Path(args.db).parent.mkdir(parents=True, exist_ok=True)
    con = connect(args.db)
    if args.command == "fetch":
        fetch(con, args.repo, args.limit)
    elif args.command == "ingest":
        for path in args.files:
            print("run %d: %d records" % ingest(con, path))
    else:
        print(report(con, args.nights, (args.baseline, args.aux_new_os, args.aux_old_os), args.min_failures))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
