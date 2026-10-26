#!/usr/bin/env python3
"""Run a command until it succeeds N consecutive times."""

from __future__ import annotations

import argparse
import csv
import lzma
import os
import shlex
import subprocess
import sys
import tarfile
import tempfile
from datetime import datetime, timezone
from pathlib import Path
from typing import Sequence


def utc_now() -> str:
    return datetime.now(timezone.utc).isoformat(timespec="seconds").replace(
        "+00:00", "Z"
    )


def positive_integer(value: str) -> int:
    try:
        number = int(value)
    except ValueError as error:
        raise argparse.ArgumentTypeError("must be an integer") from error
    if number < 1:
        raise argparse.ArgumentTypeError("must be greater than zero")
    return number


def parse_args(argv: Sequence[str]) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Run COMMAND until it succeeds N consecutive times. A failure resets "
            "the consecutive-success count to zero."
        )
    )
    parser.add_argument(
        "-n",
        "--runs",
        type=positive_integer,
        default=100,
        help="required consecutive successful runs (default: 100)",
    )
    parser.add_argument(
        "-o",
        "--output",
        type=Path,
        help="output archive (default: test-logs-<UTC timestamp>.tar.xz)",
    )
    parser.add_argument(
        "command",
        nargs=argparse.REMAINDER,
        help="command and arguments; place them after --",
    )
    args = parser.parse_args(argv)

    if args.command and args.command[0] == "--":
        args.command = args.command[1:]
    if not args.command:
        parser.error("a command is required; put it after --")
    return args


def capture_launch_environment() -> tuple[dict[str, str], Path, Path | None]:
    """Capture one stable launch environment for every command attempt."""
    environment = os.environ.copy()
    working_directory = Path.cwd().resolve()

    virtual_environment_value = environment.get("VIRTUAL_ENV")
    if virtual_environment_value:
        virtual_environment = Path(virtual_environment_value).resolve()
    elif sys.prefix != getattr(sys, "base_prefix", sys.prefix):
        # The runner was invoked with a venv Python without first activating it.
        virtual_environment = Path(sys.prefix).resolve()
        environment["VIRTUAL_ENV"] = str(virtual_environment)
    else:
        virtual_environment = None

    if virtual_environment is not None:
        scripts_directory = virtual_environment / (
            "Scripts" if os.name == "nt" else "bin"
        )
        scripts_text = str(scripts_directory)
        current_path = environment.get("PATH", "")
        path_entries = current_path.split(os.pathsep) if current_path else []

        # Keep the venv first and avoid accumulating duplicate entries.
        normalized_scripts = os.path.normcase(os.path.abspath(scripts_text))
        path_entries = [
            entry
            for entry in path_entries
            if os.path.normcase(os.path.abspath(entry)) != normalized_scripts
        ]
        environment["PATH"] = os.pathsep.join([scripts_text, *path_entries])

    return environment, working_directory, virtual_environment


def write_metadata(
    path: Path,
    command: Sequence[str],
    working_directory: Path,
    virtual_environment: Path | None,
    target: int,
    attempts: int,
    streak: int,
    total_successes: int,
    failures: int,
) -> None:
    path.write_text(
        "\n".join(
            [
                f"Command: {shlex.join(command)}",
                f"Working directory: {working_directory}",
                f"Runner Python: {sys.executable}",
                f"Virtual environment: {virtual_environment or 'none'}",
                f"Required consecutive successes: {target}",
                f"Attempts completed: {attempts}",
                f"Current consecutive-success streak: {streak}",
                f"Total successful attempts: {total_successes}",
                f"Failed attempts: {failures}",
                "",
            ]
        ),
        encoding="utf-8",
    )


def create_archive(log_directory: Path, archive: Path) -> None:
    """Atomically replace archive with all logs using maximum xz compression."""
    descriptor, temporary_name = tempfile.mkstemp(
        dir=archive.parent,
        prefix=f".{archive.name}.",
        suffix=".tmp",
    )
    os.close(descriptor)
    temporary_archive = Path(temporary_name)

    try:
        with tarfile.open(
            temporary_archive,
            mode="w:xz",
            preset=9 | lzma.PRESET_EXTREME,
        ) as tar:
            for item in sorted(log_directory.iterdir()):
                tar.add(item, arcname=Path("test-logs") / item.name)
        os.replace(temporary_archive, archive)
    except BaseException:
        temporary_archive.unlink(missing_ok=True)
        raise


def main(argv: Sequence[str] | None = None) -> int:
    args = parse_args(sys.argv[1:] if argv is None else argv)
    command: list[str] = args.command
    target: int = args.runs
    command_environment, working_directory, virtual_environment = (
        capture_launch_environment()
    )

    timestamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
    archive = (args.output or Path(f"test-logs-{timestamp}.tar.xz")).resolve()
    archive.parent.mkdir(parents=True, exist_ok=True)

    if archive.exists():
        print(f"Error: refusing to overwrite existing archive: {archive}", file=sys.stderr)
        return 2

    streak = 0
    attempts = 0
    total_successes = 0
    failures = 0

    print(f"Running until {target} consecutive successful run(s)...")
    print(f"Command: {shlex.join(command)}")
    print(f"Working directory: {working_directory}")
    if virtual_environment is not None:
        print(f"Virtual environment: {virtual_environment}")

    try:
        with tempfile.TemporaryDirectory(
            prefix="consecutive-test-", dir=archive.parent
        ) as directory:
            log_directory = Path(directory)
            summary_path = log_directory / "summary.tsv"
            metadata_path = log_directory / "metadata.txt"

            with summary_path.open("w", encoding="utf-8", newline="") as summary:
                writer = csv.writer(summary, delimiter="\t", lineterminator="\n")
                writer.writerow(
                    [
                        "attempt",
                        "exit_status",
                        "streak_before",
                        "streak_after",
                        "total_successes",
                        "failures",
                        "started_utc",
                        "finished_utc",
                        "log_file",
                    ]
                )
                summary.flush()

                while streak < target:
                    attempts += 1
                    streak_before = streak
                    started = utc_now()
                    log_name = f"attempt_{attempts:06d}.log"
                    log_path = log_directory / log_name

                    with log_path.open("w", encoding="utf-8") as log:
                        log.write(f"Attempt: {attempts}\n")
                        log.write(f"Started (UTC): {started}\n")
                        log.write(f"Command: {shlex.join(command)}\n\n")
                        log.write(f"Working directory: {working_directory}\n")
                        log.write(
                            f"Virtual environment: {virtual_environment or 'none'}\n\n"
                        )
                        log.flush()

                        try:
                            completed = subprocess.run(
                                command,
                                stdout=log,
                                stderr=subprocess.STDOUT,
                                check=False,
                                cwd=working_directory,
                                env=command_environment,
                            )
                            status = completed.returncode
                        except OSError as error:
                            log.write(f"Could not execute command: {error}\n")
                            status = 127
                            command_error = error
                        else:
                            command_error = None

                        finished = utc_now()
                        log.write(f"\nFinished (UTC): {finished}\n")
                        log.write(f"Exit status: {status}\n")

                    if status == 0:
                        streak += 1
                        total_successes += 1
                    else:
                        failures += 1
                        streak = 0

                    writer.writerow(
                        [
                            attempts,
                            status,
                            streak_before,
                            streak,
                            total_successes,
                            failures,
                            started,
                            finished,
                            log_name,
                        ]
                    )
                    summary.flush()

                    write_metadata(
                        metadata_path,
                        command,
                        working_directory,
                        virtual_environment,
                        target,
                        attempts,
                        streak,
                        total_successes,
                        failures,
                    )
                    create_archive(log_directory, archive)

                    if status == 0:
                        print(
                            f"Attempt {attempts}: succeeded; consecutive successes "
                            f"{streak}/{target} (total successes: {total_successes})."
                        )
                    else:
                        print(
                            f"Attempt {attempts}: FAILED (exit {status}); "
                            f"successful runs before reset: {streak_before}; "
                            f"streak reset to 0 (total successes: {total_successes})."
                        )

                    print(f"Archive updated: {archive}")

                    if command_error is not None:
                        print(f"Error: could not execute command: {command_error}", file=sys.stderr)
                        return 2

    except KeyboardInterrupt:
        print("\nInterrupted. The archive contains every completed attempt.", file=sys.stderr)
        return 130
    except (OSError, tarfile.TarError) as error:
        print(f"Error: could not update archive: {error}", file=sys.stderr)
        return 2

    print(
        f"SUCCESS: reached {target} consecutive successful runs after "
        f"{attempts} attempt(s)."
    )
    print(f"Final archive: {archive}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
