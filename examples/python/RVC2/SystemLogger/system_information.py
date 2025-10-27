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

# Create pipeline
pipeline = dai.Pipeline()

# Create system logger node
sysLog = pipeline.create(dai.node.SystemLogger)
sysLog.setRate(1)  # 1 Hz

# Create output
sysLogQueue = sysLog.out.createOutputQueue(maxSize=4, blocking=False)

# Start pipeline
pipeline.start()
while pipeline.isRunning():
    sysInfo = sysLogQueue.get() # Blocking call, will wait until a new data has arrived
    printSystemInformation(sysInfo)
