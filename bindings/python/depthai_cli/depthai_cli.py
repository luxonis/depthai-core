import os
from pathlib import Path
import subprocess

here = os.path.dirname(os.path.realpath(__file__))

if os.path.exists(os.path.join(here, "cam_test.py")):  # Installed package
    CAM_TEST_PATH = Path(here) / "cam_test.py"
else:
    CAM_TEST_PATH = (
        Path(here) / ".." / "utilities" / "cam_test.py"
    )  # Execution from source
CAM_TEST_PATH = str(CAM_TEST_PATH)

if os.path.exists(os.path.join(here, "flash_network_bootloader.py")):  # Installed package
    FLASH_NETWORK_BOOTLOADER_PATH = Path(here) / "flash_network_bootloader.py"
else:
    FLASH_NETWORK_BOOTLOADER_PATH = (
        Path(here)
        / ".."
        / ".."
        / ".."
        / "utilities"
        / "flash_network_bootloader.py"
    )  # Execution from source


def cli(argv=None) -> int:
    import argparse
    import sys
    import depthai as dai
    parser = argparse.ArgumentParser(description="DepthAI CLI", add_help=True)
    parser.add_argument("-v", "--version", action="store_true", help="Print version and exit.")
    parser.add_argument("-l", "--list-devices", action="store_true", help="List connected devices.")
    parser.add_argument("-f", "--flash", action="store_true",
                        help="Safely update an RVC2 NETWORK bootloader; remaining arguments are passed to the flashing command.",
    )
    subparsers = parser.add_subparsers(dest="command", help="Sub-commands")
    cam_test_parser = subparsers.add_parser("cam_test", help="Commands and options for cam_test", add_help=False)
    cam_test_parser.add_argument("args", nargs=argparse.REMAINDER, help="Arguments to pass to cam_test")

    cli_args = sys.argv[1:] if argv is None else list(argv)

    if cli_args and cli_args[0] in ("-f", "--flash"):  # Before parsing: --flash --help must reach the flash command
        return subprocess.run(
            [sys.executable, str(FLASH_NETWORK_BOOTLOADER_PATH)] + cli_args[1:]
        ).returncode

    args, unknown_args = parser.parse_known_args(cli_args)  # parse_args would also parse subparser REMAINDER args
    if args.flash:
        return subprocess.run(
            [sys.executable, str(FLASH_NETWORK_BOOTLOADER_PATH)] + unknown_args
        ).returncode
    elif args.command == "cam_test":
        cam_test_path = CAM_TEST_PATH
        return subprocess.run([sys.executable, cam_test_path] + cam_test_parser.parse_args().args[1:]).returncode
    elif unknown_args:
        parser.error(f"Unrecognized arguments: {unknown_args}") # handles exit internally
    elif args.version:
        print(dai.__version__)
        return 0
    elif args.list_devices:
        print(dai.Device.getAllConnectedDevices())
        return 0
    else:
        parser.print_help()
        return 1


if __name__ == "__main__":
    exit(cli())
