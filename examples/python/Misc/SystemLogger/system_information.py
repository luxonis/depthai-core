#!/usr/bin/env python3

import depthai as dai

def printSystemInformation(info: dai.SystemInformation):
    m = 1024 * 1024 # MiB
    print(f"Ddr used / total - {info.ddrMemoryUsage.used / m:.2f} / {info.ddrMemoryUsage.total / m:.2f} MiB")
    print(f"Cmx used / total - {info.cmxMemoryUsage.used / m:.2f} / {info.cmxMemoryUsage.total / m:.2f} MiB")
    print(f"LeonCss heap used / total - {info.leonCssMemoryUsage.used / m:.2f} / {info.leonCssMemoryUsage.total / m:.2f} MiB")
    print(f"LeonMss heap used / total - {info.leonMssMemoryUsage.used / m:.2f} / {info.leonMssMemoryUsage.total / m:.2f} MiB")
    t = info.chipTemperature
    print(f"Chip temperature - average: {t.average:.2f}, css: {t.css:.2f}, mss: {t.mss:.2f}, upa: {t.upa:.2f}, dss: {t.dss:.2f}")
    print(f"Cpu usage - Leon CSS: {info.leonCssCpuUsage.average * 100:.2f}%, Leon MSS: {info.leonMssCpuUsage.average * 100:.2f} %")
    print("----------------------------------------")

def printSystemInformationRVC4(info: dai.SystemInformationRVC4):
    m = 1024 * 1024 # MiB
    print(f"Ddr used / total - {info.ddrMemoryUsage.used / m:.2f} / {info.ddrMemoryUsage.total / m:.2f} MiB")
    print(f"Device process memory usage: {info.processMemoryUsage / 1024:.2f} MiB")
    print(f"Average Cpu usage: {info.cpuAvgUsage.average * 100:.2f}%")

    t = info.chipTemperature
    print(f"Chip temperature - average: {t.average:.2f}, cpuss: {t.cpuss:.2f}, gpuss: {t.gpuss:.2f}, mdmss: {t.mdmss:.2f}, video: {t.video:.2f}, ddr: {t.ddr:.2f}, camera: {t.camera:.2f}")
    print("----------------------------------------")

# Create pipeline
pipeline = dai.Pipeline()

# Create system logger node
sysLog = pipeline.create(dai.node.SystemLogger)
sysLog.setRate(1)  # 1 Hz

# Create output
sysLogQueue = sysLog.out.createOutputQueue(maxSize=4, blocking=False)

# Start pipeline
pipeline.start()
platform = pipeline.getDefaultDevice().getPlatform()
while pipeline.isRunning():
    sysInfo = sysLogQueue.get() # Blocking call, will wait until a new data has arrived
    if platform == dai.Platform.RVC2:
        printSystemInformation(sysInfo)
    else:
        printSystemInformationRVC4(sysInfo)
