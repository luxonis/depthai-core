#!/usr/bin/env python3

from pathlib import Path
import depthai as dai
import argparse

calibBackUpFile = str(
    (Path(__file__).parent / Path("depthai_calib_backup.json")).resolve().absolute()
)

parser = argparse.ArgumentParser()
parser.add_argument("calibJsonFile", help="Path to calibration file in json")
args = parser.parse_args()

device = dai.Device(dai.UsbSpeed.HIGH)
deviceCalib = device.readCalibration()
deviceCalib.eepromToJsonFile(calibBackUpFile)
print("Calibration Data on the device is backed up at:")
print(calibBackUpFile)
calibData = dai.CalibrationHandler(args.calibJsonFile)

try:
    device.flashCalibration(calibData)
    print("Successfully flashed calibration")
except Exception as ex:
    print(f"Failed flashing calibration: {ex}")
