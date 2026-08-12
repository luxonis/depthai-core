import importlib.util
import shutil
import subprocess
import sys
from pathlib import Path


DEPTHAI_CLI_PATH = Path(__file__).resolve().parents[1] / "depthai_cli" / "depthai_cli.py"
SPEC = importlib.util.spec_from_file_location("depthai_cli_under_test", DEPTHAI_CLI_PATH)
assert SPEC is not None
assert SPEC.loader is not None
depthai_cli = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(depthai_cli)


def test_flash_runs_network_bootloader_script(monkeypatch):
    completed = subprocess.CompletedProcess([], 7)

    def run(_command):
        return completed

    monkeypatch.setattr(depthai_cli.subprocess, "run", run)

    result = depthai_cli.cli(["--flash"])

    assert result == 7


def test_flash_forwards_bootloader_arguments(monkeypatch):
    commands = []

    def run(command):
        commands.append(command)
        return subprocess.CompletedProcess(command, 0)

    monkeypatch.setattr(depthai_cli.subprocess, "run", run)

    result = depthai_cli.cli(
        ["--flash", "--device", "10.12.234.161", "--yes"]
    )

    assert result == 0
    assert commands == [
        [
            sys.executable,
            str(depthai_cli.FLASH_NETWORK_BOOTLOADER_PATH),
            "--device",
            "10.12.234.161",
            "--yes",
        ]
    ]
    assert Path(commands[0][1]).is_file()


def test_flash_uses_script_bundled_with_installed_cli(tmp_path):
    package_dir = tmp_path / "depthai_cli"
    package_dir.mkdir()
    installed_cli_path = package_dir / "depthai_cli.py"
    installed_flash_path = package_dir / "flash_network_bootloader.py"
    shutil.copyfile(DEPTHAI_CLI_PATH, installed_cli_path)
    installed_flash_path.touch()

    installed_spec = importlib.util.spec_from_file_location(
        "installed_depthai_cli_under_test", installed_cli_path
    )
    assert installed_spec is not None
    assert installed_spec.loader is not None
    installed_cli = importlib.util.module_from_spec(installed_spec)
    installed_spec.loader.exec_module(installed_cli)

    assert installed_cli.FLASH_NETWORK_BOOTLOADER_PATH == installed_flash_path
