#!/usr/bin/env python3
"""Test dot/flood IR LEDs with mono camera preview."""

import cv2
import depthai as dai

DOT_STEP = 0.1
FLOOD_STEP = 0.1


def print_drivers(device):
    print("Detected IR drivers:")
    for name, bus, addr in device.getIrDrivers():
        print(f"  name={name}  bus={bus}  addr=0x{addr:02x}")


def apply_leds(device, dot, flood, mask):
    ok_dot = device.setIrLaserDotProjectorIntensity(dot, mask)
    ok_flood = device.setIrFloodLightIntensity(flood, mask)
    print(f"dot={dot:.1f}  flood={flood:.1f}  mask={mask}  dot_ok={ok_dot}  flood_ok={ok_flood}")


def main():
    device = dai.Device()
    print(f"Connected to: {device.getDeviceName()}")
    print_drivers(device)

    with dai.Pipeline(device) as pipeline:
        mono_left = pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_B)
        # mono_right = pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_C)

        left_out = mono_left.requestFullResolutionOutput(type=dai.ImgFrame.Type.NV12)
        # right_out = mono_right.requestFullResolutionOutput(type=dai.ImgFrame.Type.NV12)

        left_queue = left_out.createOutputQueue()
        # right_queue = right_out.createOutputQueue()

        dot_intensity = 0.0
        flood_intensity = 0.0
        mask = -1  # all drivers/sides

        pipeline.start()
        apply_leds(device, dot_intensity, flood_intensity, mask)

        print("Controls:")
        print("  W/S : increase/decrease dot projector intensity")
        print("  A/D : increase/decrease flood light intensity")
        print("  1   : apply only to LEFT (mask 0x1)")
        print("  2   : apply only to RIGHT (mask 0x2)")
        print("  3   : apply to ALL (mask -1)")
        print("  P   : re-print IR drivers")
        print("  Q   : quit")

        while pipeline.isRunning():
            left = left_queue.get()
            # right = right_queue.get()
            assert isinstance(left, dai.ImgFrame)
            # assert isinstance(right, dai.ImgFrame)

            cv2.imshow("left", left.getCvFrame())
            # cv2.imshow("right", right.getCvFrame())

            key = cv2.waitKey(1)
            changed = False

            if key == ord('q') or key == 27:
                break
            elif key == ord('w'):
                dot_intensity = min(dot_intensity + DOT_STEP, 1.0)
                changed = True
            elif key == ord('s'):
                dot_intensity = max(dot_intensity - DOT_STEP, 0.0)
                changed = True
            elif key == ord('a'):
                flood_intensity = min(flood_intensity + FLOOD_STEP, 1.0)
                changed = True
            elif key == ord('d'):
                flood_intensity = max(flood_intensity - FLOOD_STEP, 0.0)
                changed = True
            elif key == ord('1'):
                mask = 0x1
                changed = True
            elif key == ord('2'):
                mask = 0x2
                changed = True
            elif key == ord('3'):
                mask = -1
                changed = True
            elif key == ord('p'):
                print_drivers(device)

            if changed:
                apply_leds(device, dot_intensity, flood_intensity, mask)

        pipeline.stop()


if __name__ == "__main__":
    main()
