import time

import depthai as dai


def connect_two_devices():
    infos = dai.Device.getAllAvailableDevices()
    devices = []

    for info in infos:
        try:
            device = dai.Device(info)
            print(f"Connected to {device.getDeviceId()} ({device.getPlatformAsString()})")
            devices.append(device)
            if len(devices) == 2:
                return devices
        except Exception as exc:
            print(f"Skipping {info}: {exc}")

    raise RuntimeError(f"Need at least 2 connectable devices, found {len(devices)}")


def main():
    dev1, dev2 = connect_two_devices()

    with dai.Pipeline(False) as pipeline:
        log1 = pipeline.create(dai.node.SystemLogger, dev1)
        log2 = pipeline.create(dai.node.SystemLogger, dev2)

        log1.setRate(1)
        log2.setRate(1)

        q1 = log1.out.createOutputQueue(maxSize=4, blocking=False)
        q2 = log2.out.createOutputQueue(maxSize=4, blocking=False)

        pipeline.start()
        print("Multi-device pipeline started")

        while True:
            msg1 = q1.tryGet()
            if msg1 is not None:
                print(f"{dev1.getDeviceId()}: {type(msg1).__name__}")

            msg2 = q2.tryGet()
            if msg2 is not None:
                print(f"{dev2.getDeviceId()}: {type(msg2).__name__}")

            time.sleep(0.05)


if __name__ == "__main__":
    main()
