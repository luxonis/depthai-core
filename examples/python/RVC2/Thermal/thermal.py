import depthai as dai
import cv2

THERMAL_IMAGE_BRIGHTNESS_STEP = 10

with dai.Pipeline(True) as pipeline:
    thermal = pipeline.create(dai.node.Thermal)
    thermalImgOut = thermal.color.createOutputQueue()
    thermalConfIn = thermal.inputConfig.createInputQueue()
    thermalConf = dai.ThermalConfig()

    pipeline.start()
    WINDOW_NAME = "thermal"
    cv2.namedWindow(WINDOW_NAME, cv2.WINDOW_NORMAL)
    initialRescaleDone = False
    while True:
        thermalImg = thermalImgOut.get()
        if not initialRescaleDone:
            cv2.resizeWindow(WINDOW_NAME, thermalImg.getWidth(), thermalImg.getHeight())
            initialRescaleDone = True
        cv2.imshow(WINDOW_NAME, thermalImg.getCvFrame())
        key = cv2.waitKey(1)
        changed = False
        if key == ord("q"):
            break
        elif key == ord("n"):
            if thermalConf.imageParams.brightnessLevel is None:
                thermalConf.imageParams.brightnessLevel = THERMAL_IMAGE_BRIGHTNESS_STEP
            if (
                thermalConf.imageParams.brightnessLevel - THERMAL_IMAGE_BRIGHTNESS_STEP
            ) <= THERMAL_IMAGE_BRIGHTNESS_STEP:
                thermalConf.imageParams.brightnessLevel = THERMAL_IMAGE_BRIGHTNESS_STEP
            thermalConf.imageParams.brightnessLevel -= THERMAL_IMAGE_BRIGHTNESS_STEP
            print(
                "Set image brightness level to ",
                thermalConf.imageParams.brightnessLevel,
            )
            changed = True
        elif key == ord("m"):
            if thermalConf.imageParams.brightnessLevel is None:
                thermalConf.imageParams.brightnessLevel = 0
            if (
                thermalConf.imageParams.brightnessLevel + THERMAL_IMAGE_BRIGHTNESS_STEP
            ) >= 255:
                thermalConf.imageParams.brightnessLevel = (
                    255 - THERMAL_IMAGE_BRIGHTNESS_STEP
                )
            thermalConf.imageParams.brightnessLevel += THERMAL_IMAGE_BRIGHTNESS_STEP
            print(
                "Set image brightness level to ",
                thermalConf.imageParams.brightnessLevel,
            )
            changed = True
        elif key == ord("a"):
            if thermalConf.ffcParams.autoFFC is None:
                thermalConf.ffcParams.autoFFC = True
            thermalConf.ffcParams.autoFFC = not thermalConf.ffcParams.autoFFC
            print(
                "Set auto shutter to",
                "on." if thermalConf.ffcParams.autoFFC else "off.",
            )
            changed = True
        elif key == ord("s"):
            if thermalConf.ffcParams.closeManualShutter is None:
                thermalConf.ffcParams.closeManualShutter = False
            thermalConf.ffcParams.closeManualShutter = (
                not thermalConf.ffcParams.closeManualShutter
            )
            print(
                "Closing" if thermalConf.ffcParams.closeManualShutter else "Opening",
                "manual shutter.",
            )
            changed = True

        if changed:
            thermalConfIn.send(thermalConf)
