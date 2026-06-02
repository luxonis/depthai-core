#!/usr/bin/env python3

import depthai as dai
import json

device = dai.Device(dai.UsbSpeed.HIGH)

camera_sockets = [
    dai.CameraBoardSocket.CAM_A,
    dai.CameraBoardSocket.CAM_B,
    dai.CameraBoardSocket.CAM_C,
    dai.CameraBoardSocket.CAM_D,
    dai.CameraBoardSocket.CAM_E,
    dai.CameraBoardSocket.CAM_F,
    dai.CameraBoardSocket.CAM_G,
    dai.CameraBoardSocket.CAM_H
]

for cam in camera_sockets:
    print(f'\n========== {cam.name} ==========')
    hasEeprom = device.isCBAEepromAvailable(cam)
    print(f'[{cam.name}] Is EEPROM available: {hasEeprom}')
    if hasEeprom:
        try:
            print(f'[{cam.name}] User calibration: {json.dumps(device.readCBACalibration2(cam).eepromToJson(), indent=2)}')
        except dai.EepromError as ex:
            print(f'[{cam.name}] No user calibration: {ex}')

        try:
            print(f'[{cam.name}] Factory calibration: {json.dumps(device.readFactoryCBACalibration(cam).eepromToJson(), indent=2)}')
        except dai.EepromError as ex:
            print(f'[{cam.name}] No factory calibration: {ex}')
