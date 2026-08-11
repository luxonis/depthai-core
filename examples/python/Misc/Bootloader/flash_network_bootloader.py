#!/usr/bin/env python3
"""Safely update an RVC2 device's NETWORK bootloader.

By default, this script flashes the NETWORK bootloader bundled with DepthAI to
the recoverable user bootloader slot, leaving the factory bootloader unchanged.
It checks that the connected device is running a compatible NETWORK bootloader,
asks for confirmation, reports flashing progress, and reconnects after the
device reboots to verify that the expected user bootloader is running.

An explicit bootloader image and its expected version can be supplied together.
Passing ``--factory`` instead writes the factory bootloader region, which can
require hardware recovery if interrupted or unsuccessful, so that path requires
additional confirmation and is never enabled by ``--yes`` alone.
"""

import argparse
import sys
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import depthai as dai


CONNECT_ATTEMPTS = 30
CONNECT_RETRY_DELAY_SECONDS = 1.0
VERIFY_ATTEMPTS = 30
VERIFY_RETRY_DELAY_SECONDS = 2.0


class SafetyError(RuntimeError):
    """Raised when a bootloader update fails a safety check."""


@dataclass(frozen=True)
class BootloaderState:
    version: str
    isUserBootloader: bool


def parseArgs() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Safely update an RVC2 device's user NETWORK bootloader using an explicit image "
            "or the image embedded in depthai-core. Factory flashing requires -f and two confirmations."
        ),
    )
    parser.add_argument(
        "--device",
        "-d",
        help="Device IP address, name, or device ID (defaults to the first available device)",
    )
    parser.add_argument(
        "--factory",
        "-f",
        action="store_true",
        help="Flash the factory bootloader instead of the recoverable user bootloader",
    )
    parser.add_argument(
        "--yes",
        "-y",
        action="store_true",
        help="Skip the routine user-slot confirmation (device selection and factory prompts are never skipped)",
    )
    parser.add_argument(
        "--bootloader",
        type=Path,
        help="NETWORK bootloader image to flash instead of the image embedded in depthai",
    )
    parser.add_argument(
        "--expected-version",
        dest="expectedVersion",
        help="Full version expected after flashing --bootloader (for example, 0.0.28+<commit>)",
    )
    args = parser.parse_args()

    if (args.bootloader is None) != (args.expectedVersion is None):
        parser.error("--bootloader and --expected-version must be provided together")
    if args.bootloader is not None and not args.bootloader.is_file():
        parser.error(f"bootloader image does not exist or is not a file: {args.bootloader}")

    return args


def versionString(version: Any) -> str:
    """Compare the full version, including snapshot build metadata."""
    return str(version)


def inspectDevice(bootloader: Any, factory: bool = False) -> BootloaderState:
    if bootloader.getType() != dai.DeviceBootloader.Type.NETWORK:
        raise SafetyError("Refusing to update: the connected device is not running a NETWORK bootloader.")

    if not factory and not bootloader.isUserBootloaderSupported():
        raise SafetyError(
            "Refusing to update: the installed factory bootloader does not support the recoverable "
            "user bootloader slot. Use the M8/USB recovery programming board to update this device; "
            "do not flash its factory bootloader over Ethernet."
        )

    return BootloaderState(
        version=versionString(bootloader.getVersion()),
        isUserBootloader=bootloader.isUserBootloader(),
    )


def deviceLabel(device: Any) -> str:
    if isinstance(device, str):
        return device

    name = getattr(device, "name", "")
    deviceId = getattr(device, "deviceId", "")
    if name and deviceId and name != deviceId:
        return f"{name} ({deviceId})"
    return name or deviceId or str(device)


def selectFirstDevice() -> Any:
    found, device = dai.DeviceBootloader.getFirstAvailableDevice()
    if not found:
        raise RuntimeError("No available DepthAI devices found.")

    print(f"First available device: {deviceLabel(device)}")
    if input("Is this the device you want to flash? [y/N] ").strip().lower() not in {"y", "yes"}:
        return None
    return device


def confirmFactoryAccess(device: Any) -> bool:
    print(f"WARNING: Factory bootloader flashing requested for {deviceLabel(device)}.")
    print("A failed factory bootloader update may require hardware recovery.")
    return input("Allow factory bootloader access? [y/N] ").strip().lower() in {"y", "yes"}


def confirmFlash(device: Any, installed: BootloaderState, targetVersion: str, factory: bool = False) -> bool:
    print(f"Device: {deviceLabel(device)}")
    print(f"Installed NETWORK bootloader: {installed.version} is user bootloader: {installed.isUserBootloader}")
    print(f"Target NETWORK bootloader:    {targetVersion}")
    if factory:
        print("Target: FACTORY bootloader region")
        print("WARNING: This operation can make the device unbootable and require hardware recovery.")
    else:
        print("Target: recoverable USER bootloader slot (factory bootloader will not be modified)")
    print("Do not disconnect the device or interrupt its power while updating.")
    prompt = "Flash the factory bootloader? [y/N] " if factory else "Continue? [y/N] "
    response = input(prompt).strip().lower()
    return response in {"y", "yes"}


def printProgress(progress: float, factory: bool = False) -> None:
    target = "factory" if factory else "user"
    print(f"\rUpdating {target} bootloader: {progress * 100:5.1f}%", end="", flush=True)


def connectBootloader(device: Any, factory: bool = False) -> Any:
    lastError = "device did not become available"

    for attempt in range(CONNECT_ATTEMPTS):
        try:
            return dai.DeviceBootloader(device, factory)
        except RuntimeError as exc:
            if "Specified device not found" not in str(exc):
                raise
            lastError = str(exc)

        if attempt + 1 < CONNECT_ATTEMPTS:
            print(
                f"Device is reconnecting; retrying discovery "
                f"({attempt + 1}/{CONNECT_ATTEMPTS})...",
                file=sys.stderr,
            )
            time.sleep(CONNECT_RETRY_DELAY_SECONDS)

    raise RuntimeError(f"Device did not reconnect after {CONNECT_ATTEMPTS} attempts: {lastError}")


def verifyUpdate(device: Any, targetVersion: str) -> None:
    lastError = "Device did not reconnect"

    for attempt in range(VERIFY_ATTEMPTS):
        if attempt:
            time.sleep(VERIFY_RETRY_DELAY_SECONDS)

        try:
            with dai.DeviceBootloader(device, False) as bootloader:
                state = inspectDevice(bootloader)
        except Exception as exc:
            lastError = str(exc)
            continue

        if state.version != targetVersion:
            lastError = f"Device reported version {state.version}, expected {targetVersion}"
            continue
        if not state.isUserBootloader:
            lastError = "Device rebooted into its factory bootloader instead of the updated user bootloader"
            continue

        return

    raise RuntimeError(f"Update was written but post-reboot verification failed: {lastError}")


def main() -> int:
    args = parseArgs()
    targetVersion = args.expectedVersion or versionString(dai.DeviceBootloader.getEmbeddedBootloaderVersion())
    bootloaderPath = str(args.bootloader) if args.bootloader is not None else ""

    try:
        device = args.device
        if device is None:
            device = selectFirstDevice()
            if device is None:
                print("You can select a specific device using the --device argument.")
                return 0

        if args.factory and not confirmFactoryAccess(device):
            print("Cancelled.")
            return 0

        with connectBootloader(device, args.factory) as bootloader:
            installed = inspectDevice(bootloader, args.factory)

            # if installed.version == targetVersion and installed.isUserBootloader:
            #     print(f"NETWORK bootloader is already up to date ({targetVersion}).")
            #     return 0

            if (args.factory or not args.yes) and not confirmFlash(
                device, installed, targetVersion, args.factory
            ):
                print("Cancelled.")
                return 0

            def progressCallback(progress: float) -> None:
                printProgress(progress, args.factory)

            if args.factory:
                success, error = bootloader.flashBootloader(progressCallback, bootloaderPath)
            else:
                success, error = bootloader.flashUserBootloader(progressCallback, bootloaderPath)
            print()
            if not success:
                target = "factory" if args.factory else "user"
                raise RuntimeError(f"Device rejected or failed to verify the {target} bootloader update: {error}")

        if args.factory:
            print("The device reported that the factory bootloader flash completed successfully.")
        else:
            print("Flash verification succeeded; waiting for the device to reboot and checking the running version...")
            verifyUpdate(device, targetVersion)
    except SafetyError as exc:
        print(str(exc), file=sys.stderr)
        return 2
    except Exception as exc:
        print(f"Failed to update the NETWORK bootloader: {exc}", file=sys.stderr)
        if args.factory:
            print("Factory bootloader flashing was enabled; inspect the device state before retrying.", file=sys.stderr)
        else:
            print("The factory bootloader was not modified by this script.", file=sys.stderr)
        return 1

    if args.factory:
        print(f"NETWORK factory bootloader {targetVersion} was flashed successfully.")
    else:
        print(f"NETWORK user bootloader {targetVersion} is running and verified.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
