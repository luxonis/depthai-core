#!/bin/bash

echo Removing all gadget configs

pushd /sys/kernel/config/usb_gadget/g1

echo "" > UDC

rm configs/c.1/uvc.0
rmdir configs/c.1/strings/0x409
rmdir configs/c.1

rm functions/uvc.0/streaming/header/h/*
rm functions/uvc.0/streaming/header/h1/*
rm functions/uvc.0/streaming/header/h/m
rmdir functions/uvc.0/streaming/mjpeg/m/*
rmdir functions/uvc.0/streaming/mjpeg/m1/*
rmdir functions/uvc.0/streaming/mjpeg/m
rmdir functions/uvc.0/streaming/mjpeg/m1
rmdir functions/uvc.0/streaming/uncompressed/u/*
rmdir functions/uvc.0/streaming/uncompressed/u1/*
rmdir functions/uvc.0/streaming/uncompressed/u
rmdir functions/uvc.0/streaming/uncompressed/u1

rm functions/uvc.0/streaming/class/fs/*
rm functions/uvc.0/streaming/class/hs/*
rm functions/uvc.0/streaming/class/ss/*

rmdir functions/uvc.0/streaming/header/h
rmdir functions/uvc.0/streaming/header/h1

rm functions/uvc.0/control/class/fs/*
rm functions/uvc.0/control/class/ss/*
rmdir functions/uvc.0/control/header/*

rmdir functions/*

rmdir strings/0x409

popd

rmdir /sys/kernel/config/usb_gadget/g1

if [ ! -e /sys/kernel/config/usb_gadget/g1 ]; then
    echo "Gadget successfully removed!"
else
    echo "Failed to remove gadget!"
    exit 1
fi
