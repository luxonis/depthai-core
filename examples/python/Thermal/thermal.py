import depthai as dai
import cv2

THERMAL_IMAGE_BRIGHTNESS_STEP = 10
PALETTES = list(dai.ThermalColorPalette.__members__.values())
AGC_MODES = list(dai.ThermalAgcMode.__members__.values())

with dai.Pipeline(True) as pipeline:
    thermal = pipeline.create(dai.node.Thermal)
    thermalImgOut = thermal.color.createOutputQueue()
    thermalConfIn = thermal.inputConfig.createInputQueue()
    thermalConf = dai.ThermalConfig()

    pipeline.start()
    print("Keys: p palette, g AGC mode, l fixed 20-40 deg C span, f shutter now, a auto shutter, s manual shutter, n/m brightness, q quit")
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
        elif key == ord("p"):
            palette = thermalConf.imageParams.colorPalette
            if palette is None:
                palette = dai.ThermalColorPalette.WHITE_HOT  # at start
            thermalConf.imageParams.colorPalette = PALETTES[(PALETTES.index(palette) + 1) % len(PALETTES)]
            print("Set color palette to", thermalConf.imageParams.colorPalette.name)
            changed = True
        elif key == ord("g"):
            mode = thermalConf.agcParams.mode
            thermalConf.agcParams.mode = AGC_MODES[0 if mode is None else (AGC_MODES.index(mode) + 1) % len(AGC_MODES)]
            print("Set AGC mode to", thermalConf.agcParams.mode.name)
            changed = True
        elif key == ord("l"):
            # The same temperature gets the same color in every frame: 20 deg C and colder darkest, 40 deg C and hotter brightest
            fixed = thermalConf.agcParams.linearLockMode != dai.ThermalLinearAgcLockMode.MANUAL
            thermalConf.agcParams.mode = dai.ThermalAgcMode.LINEAR
            if fixed:
                thermalConf.agcParams.linearLockMode = dai.ThermalLinearAgcLockMode.MANUAL
                thermalConf.agcParams.linearMin = 20.0
                thermalConf.agcParams.linearMax = 40.0
            else:
                thermalConf.agcParams.linearLockMode = dai.ThermalLinearAgcLockMode.AUTO
            print("Set linear AGC to", "20-40 deg C." if fixed else "the scene's range.")
            changed = True
        elif key == ord("f"):
            # One shutter cycle; sent on its own, so later configs don't repeat it
            trigger = dai.ThermalConfig()
            trigger.ffcParams.triggerFFC = True
            thermalConfIn.send(trigger)
            print("Shutter (flat field correction) triggered.")

        if changed:
            thermalConfIn.send(thermalConf)
