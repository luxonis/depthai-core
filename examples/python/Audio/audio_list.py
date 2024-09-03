#!/usr/bin/env python3

import depthai as dai

# Create pipeline
device = dai.Device()

devices = device.getAlsaDevices();

for d in devices:
    print("Name: " + d.name + "\n");
    print("Desc: " + d.desc + "\n");
    print("IOID: " + d.ioid + "\n");

