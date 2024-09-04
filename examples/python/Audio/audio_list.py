#!/usr/bin/env python3

import depthai as dai

# Create pipeline
device = dai.Device()

devices = device.getAlsaDevices();

for d in devices:
    print("Name: " + d.name + "\n");
    print("Desc: " + d.desc + "\n");
    print("IOID: " + d.ioid + "\n");

pcms = device.getAlsaPCMs();

for p in pcms:
    print("Name:  " + p.name + "\n");
    print("ID:    " + p.id+ "\n");
    print("Card#: " + p.cardNumber + "\n");
    print("Dev#:  " + p.deviceNumer + "\n");

