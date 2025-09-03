#!/usr/bin/env python3

import depthai as dai

device = dai.Device(dai.UsbSpeed.HIGH)
try:
    device.factoryResetCalibration()
    print(f'Factory reset calibration OK')
except Exception as ex:
    print(f'Factory reset calibration FAIL: {ex}')
