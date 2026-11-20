#!/usr/bin/env python3
"""Report functional results and compare the existing RVC2 streaming workload."""

import json
from pathlib import Path
import re
import statistics
import sys
import xml.etree.ElementTree as ET


def read_stress(path):
    samples = {}
    speeds = set()
    duration = 0
    with path.open(errors="replace") as stream:
        for line in stream:
            match = re.search(r"\[(\d+)s\] Usb speed (.*?) FPS: (.*)", line)
            if match and int(match[1]) >= 30:
                duration = max(duration, int(match[1]))
                speeds.add(match[2].strip())
                for name, value in re.findall(r"(\w+):\s*([\d.]+)", match[3]):
                    samples.setdefault(name + " FPS", []).append(float(value))
            match = re.search(r"(Ddr|Cmx|LeonCss heap|LeonMss heap) used / total - ([\d.]+)", line)
            if match:
                samples.setdefault(match[1] + " MiB", []).append(float(match[2]))
            match = re.search(r"Cpu usage - Leon CSS: ([\d.]+) %, Leon MSS: ([\d.]+) %", line)
            if match:
                samples.setdefault("Leon CSS CPU %", []).append(float(match[1]))
                samples.setdefault("Leon MSS CPU %", []).append(float(match[2]))
            match = re.search(r"Chip temperature - average: ([\d.]+)", line)
            if match:
                samples.setdefault("Temperature C", []).append(float(match[1]))
    return {
        "duration_seconds": duration,
        "usb_speeds": sorted(speeds),
        "metrics": {
            name: {"samples": len(values), "mean": statistics.mean(values), "min": min(values), "max": max(values)}
            for name, values in samples.items()
        },
    }


def main(directory):
    root = Path(directory)
    options = json.loads((root / "options.json").read_text(encoding="utf-8-sig"))
    raw = json.loads((root / "exit-codes.json").read_text(encoding="utf-8-sig"))
    mode = options.get("mode", "libusb-ab")
    results = {name: value if isinstance(value, dict) else {"boot": value} for name, value in raw.items()}
    issues = []
    print((root / "summary.md").read_text(encoding="utf-8-sig"))
    failed_tests = {}
    for variant in ("unpatched", "patched"):
        failed_tests[variant] = []
        for phase, code in results[variant].items():
            if code and (variant == "patched" or mode == "suite"):
                issues.append(f"{variant} {phase} failed (CTest exit {code}).")
            report = root / variant / f"{phase}.xml"
            if report.exists():
                for case in ET.parse(report).iter("testcase"):
                    if case.find("failure") is not None or case.find("error") is not None:
                        failed_tests[variant].append(case.get("name"))
    new_failures = sorted(set(failed_tests["patched"]) - set(failed_tests["unpatched"]))
    for variant, names in failed_tests.items():
        if names:
            print(f"Failing {variant} tests: " + ", ".join(names) + "\n")
    if new_failures:
        print("New failing tests with the patch: " + ", ".join(new_failures) + "\n")

    if mode == "suite":
        measurements = {variant: read_stress(root / variant / "stress.log") for variant in results}
        for variant, data in measurements.items():
            if data["duration_seconds"] < options["stress_seconds"] - 10:
                issues.append(f"{variant} did not record the requested streaming duration.")
        limit = float(options["max_fps_drop_percent"])
        print(f"Mean stream FPS comparison: maximum allowed drop {limit:g}%. FPS samples before 30 seconds excluded.\n")
        print("| Metric | Unpatched mean | Patched mean | Change |")
        print("| --- | ---: | ---: | ---: |")
        baseline = measurements["unpatched"]["metrics"]
        patched = measurements["patched"]["metrics"]
        expected = {name + " FPS" for name in ("out1", "out2", "out3", "imgFrame", "inDet", "depth", "edgeLefts", "edgeRights")}
        resources = {"Ddr MiB", "Cmx MiB", "LeonCss heap MiB", "LeonMss heap MiB", "Leon CSS CPU %", "Leon MSS CPU %", "Temperature C"}
        for name in sorted(set(baseline) | set(patched) | expected | resources):
            before, after = baseline.get(name), patched.get(name)
            if before is None or after is None or min(before["samples"], after["samples"]) < 3:
                issues.append(f"Insufficient measurements for {name}.")
                continue
            change = (after["mean"] / before["mean"] - 1) * 100 if before["mean"] else None
            delta = f"{change:+.2f}%" if change is not None else "n/a"
            print(f"| {name} | {before['mean']:.2f} | {after['mean']:.2f} | {delta} |")
            if name in expected:
                if before["mean"] <= 0:
                    issues.append(f"Invalid baseline stream rate: {name}.")
                elif after["mean"] < before["mean"] * (1 - limit / 100):
                    issues.append(f"Stream rate regression: {name} ({delta}).")
        speeds = [measurements[variant]["usb_speeds"] for variant in ("unpatched", "patched")]
        print(f"\nUSB speed: unpatched={speeds[0]}, patched={speeds[1]}.\n")
        if len(speeds[0]) != 1 or speeds[0] != speeds[1]:
            issues.append("USB link speeds differ or were not recorded; comparison is invalid.")
        print("Device CPU, memory and temperature include the entire run; min/max values are in performance.json. Review these metrics separately.\n")
        (root / "performance.json").write_text(json.dumps({"measurements": measurements, "issues": issues, "failed_tests": failed_tests, "new_failures": new_failures}, indent=2))
    elif not issues:
        if any(results["unpatched"].values()):
            print("Unpatched failed and patched passed; inspect the baseline log for the expected failure.\n")
        else:
            print("Both variants passed; the boot/discovery workload did not reproduce the issue.\n")
    if issues:
        print("\n".join("- " + issue for issue in issues))
    else:
        print("Functional and mean-FPS checks passed." if mode == "suite" else "No patched boot/discovery failures.")
    return int(bool(issues))


if __name__ == "__main__":
    sys.exit(main(sys.argv[1]))
