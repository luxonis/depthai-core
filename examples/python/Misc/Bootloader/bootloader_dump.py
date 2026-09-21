#!/usr/bin/env python3

import depthai as dai


found, deviceInfo = dai.DeviceBootloader.getFirstAvailableDevice()
if not found:
    raise RuntimeError("No available device found")

deviceState = str(deviceInfo.state).replace("XLinkDeviceState.", "")

with dai.DeviceBootloader(deviceInfo) as bootloader:
    version = bootloader.getVersion()
    userBootloader = bootloader.isUserBootloader()

bootloaderType = "Factory flashed bootloader" if not userBootloader else "User flashed bootloader"

print(deviceInfo)
print(f"Bootloader version: {bootloaderType} {version}")
print(f"Embedded depthai bootloader version: {dai.DeviceBootloader.getEmbeddedBootloaderVersion()}")
