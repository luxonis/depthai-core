#!/usr/bin/env python3

import depthai as dai
import json

device = dai.Device(dai.UsbSpeed.HIGH)
print(f'Is EEPROM available: {device.isEepromAvailable()}')

# User calibration
try:
    print(f'User calibration: {json.dumps(device.readCalibration2().eepromToJson(), indent=2)}')
except Exception as ex:
    print(f'No user calibration: {ex}')

# Factory calibration
try:
    print(f'Factory calibration: {json.dumps(device.readFactoryCalibration().eepromToJson(), indent=2)}')
except Exception as ex:
    print(f'No factory calibration: {ex}')

print(f'\nUser calibration raw: {device.readCalibrationRaw().hex(" ")}')
print(f'\nFactory calibration raw: {device.readFactoryCalibrationRaw().hex(" ")}')

camera_sockets = [
    dai.CameraBoardSocket.CAM_A,
    dai.CameraBoardSocket.CAM_B,
    dai.CameraBoardSocket.CAM_C,
    dai.CameraBoardSocket.CAM_D,
]

for cam in camera_sockets:
    print(f'\n========== {cam.name} ==========')
    hasEeprom = device.isEepromAvailable(cam)
    print(f'[{cam.name}] Is EEPROM available: {hasEeprom}')
    if hasEeprom:
        try:
            print(f'[{cam.name}] User calibration: {json.dumps(device.readCalibration2(cam).eepromToJson(), indent=2)}')
        except Exception as ex:
            print(f'[{cam.name}] No user calibration: {ex}')

        try:
            print(f'[{cam.name}] Factory calibration: {json.dumps(device.readFactoryCalibration(cam).eepromToJson(), indent=2)}')
        except Exception as ex:
            print(f'[{cam.name}] No factory calibration: {ex}')

        print(f'\n[{cam.name}] User calibration raw: {device.readCalibrationRaw(cam).hex(" ")}')
        print(f'\n[{cam.name}] Factory calibration raw: {device.readFactoryCalibrationRaw(cam).hex(" ")}')


