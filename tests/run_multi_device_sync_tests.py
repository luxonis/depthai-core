#!/usr/bin/env python3

import adbutils
import argparse
import datetime
import time
import os
import subprocess
import signal
from threading import Event

interrupted = Event()

def signal_handler(signal, frame):
    interrupted.set()
    print("Received SIGINT, exiting...")


def enablePTPonCamera(device: adbutils.AdbDevice, sync_frames: bool, is_master: bool, ptp_domain: int):
    devicename = device.serial
    print(f"Enabling PTP on {devicename}")

    cmd = f"sed -i -E 's/^\\(domainNumber[[:space:]]+?\\)[0-9]+?$/\\\\1{ptp_domain}/' /etc/linuxptp/ptp4l.conf"
    ret = device.shell2(cmd, v2=True)
    if ret.returncode != 0:
        raise RuntimeError(f"{devicename} Failed to set PTP domain to {ptp_domain}: {ret.stderr}")

    role = "master" if is_master else "slave"
    cmd = f"luxonis-ptp-config mode {role}"
    ret = device.shell2(cmd, v2=True)
    if ret.returncode != 0:
        raise RuntimeError(f"{devicename} Failed to set PTP mode to {role}: {ret.stderr}")

    cmd = f"luxonis-ptp-config sync_frames {"true" if sync_frames else "false"}"
    ret = device.shell2(cmd, v2=True)
    if ret.returncode != 0:
        raise RuntimeError(f"{devicename} Failed to set PTP sync_frames to {sync_frames}: {ret.stderr}")

    cmd = "luxonis-ptp-config enable"
    ret = device.shell2(cmd, v2=True)
    if ret.returncode != 0:
        raise RuntimeError(f"{devicename} Failed to enable PTP: {ret.stderr}")

def disablePTPonCamera(device: adbutils.AdbDevice):
    devicename = device.serial
    print(f"Disabling PTP on {devicename}")
    cmd = "luxonis-ptp-config disable"
    ret = device.shell2(cmd, v2=True)
    if ret.returncode != 0:
        raise RuntimeError(f"{devicename} Failed to disable PTP: {ret.stderr}")

def enablePTPonAllDevices(devices: list[adbutils.AdbDevice], sync_frames: bool, ptp_domain: int):
    print(f"Found {len(devices)} devices")
    print(f"PTP domain: {ptp_domain}")
    print("Enabling PTP on all devices")
    for idx, device in enumerate(devices):
        if idx == 0:
            enablePTPonCamera(device, sync_frames, True, ptp_domain)
        else:
            enablePTPonCamera(device, sync_frames, False, ptp_domain)

        ret = device.shell2("reboot")
        if ret.returncode != 0:
            raise RuntimeError(f"Failed to reboot {device.serial}: {ret.stderr}")

def disablePTPonAllDevices(devices: list[adbutils.AdbDevice]):
    print(f"Found {len(devices)} devices")
    print("Disabling PTP on all devices")
    for device in devices:
        try:
            disablePTPonCamera(device)
        except RuntimeError as e:
            print(f"Failed to disable PTP on {device.serial}: {e}")

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--fsync",
        action="store_true",
        required=False,
    )

    parser.add_argument(
        "--ptp",
        action="store_true",
        required=False,
    )

    args = parser.parse_args()
    if args.fsync:
        sync_frames = False
        num_devices = 4
        test_executable = "multi_device_fsync_test"
    elif args.ptp:
        sync_frames = True
        num_devices = 3
        test_executable = "multi_device_ptp_test"
    else:
        raise RuntimeError("Must specify either --fsync or --ptp")

    signal.signal(signal.SIGINT, signal_handler)

    timeout_sec = 5*60 # 5 minutes

    print(f"Waiting for all {num_devices} devices to come online...")
    start_time = datetime.datetime.now()
    while True:
        if interrupted.is_set():
            raise RuntimeError("Interrupted by SIGINT")
        devices = adbutils.adb.device_list()
        if len(devices) == num_devices:
            break
        if datetime.datetime.now() - start_time > datetime.timedelta(seconds=timeout_sec):
            raise RuntimeError("Timeout waiting for all devices to come online")
        time.sleep(1)

    print("All devices are online")
    try:
        enablePTPonAllDevices(devices, sync_frames, 111)

        print(f"Waiting for all {num_devices} devices to come online...")
        start_time = datetime.datetime.now()
        while True:
            if interrupted.is_set():
                raise RuntimeError("Interrupted by SIGINT")
            devices = adbutils.adb.device_list()
            if len(devices) == num_devices:
                break
            if datetime.datetime.now() - start_time > datetime.timedelta(seconds=timeout_sec):
                raise RuntimeError("Timeout waiting for all devices to come online")
            time.sleep(1)

        print("All devices are online")
        print("Sleeping 2 minutes for PTP to settle")
        time.sleep(120)

        try:
            print("Running tests...")
            envvars = os.environ.copy()
            envvars["DEPTHAI_PROTOCOL"] = "tcpip"

            cmd = f"../build/tests/{test_executable}"
            subprocess.run(cmd, env=envvars, shell=True, check=True)
        except Exception as e:
            print(f"Failed to run tests: {e}")
            raise e
    finally:
        disablePTPonAllDevices(devices)

if __name__ == "__main__":
    main()