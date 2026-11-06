import depthai as dai
import cv2
import numpy as np

mouseX, mouseY = 0, 0


def onMouse(event, x, y, *args):
    global mouseX, mouseY
    mouseX = x
    mouseY = y


# Thermal camera

with dai.Pipeline(True) as pipeline:
    thermal = pipeline.create(dai.node.Thermal)
    # Output raw: FP16 temperature data (degrees Celsius)
    qTemperature = thermal.temperature.createOutputQueue()
    # Output color: YUV422i image data
    qColor = thermal.color.createOutputQueue()
    pipeline.start()
    MAGMA_WINDOW_NAME = "Colorized Temperature"
    IMAGE_WINDOW_NAME = "Thermal image"
    # Scale 4x and position one next to another
    cv2.namedWindow(MAGMA_WINDOW_NAME, cv2.WINDOW_NORMAL)
    cv2.namedWindow(IMAGE_WINDOW_NAME, cv2.WINDOW_NORMAL)
    initialRescaleAndPositionDone = False

    while True:
        inTemperature = qTemperature.get()
        inColor = qColor.get()

        thermalData = (
            inTemperature.getData()
            .view(dtype=np.float16)
            .reshape((inTemperature.getHeight(), inTemperature.getWidth()))
            .astype(np.float32)
        )
        normalizedThermalData = cv2.normalize(thermalData, None, 0, 1, cv2.NORM_MINMAX)
        normalizedThermalData = (normalizedThermalData * 255).astype(np.uint8)
        colormappedFrame = cv2.applyColorMap(normalizedThermalData, cv2.COLORMAP_MAGMA)
        if not initialRescaleAndPositionDone:
            cv2.moveWindow(MAGMA_WINDOW_NAME, 0, 0)
            width, height = colormappedFrame.shape[1], colormappedFrame.shape[0]
            cv2.resizeWindow(MAGMA_WINDOW_NAME, width * 4, height * 4)
            cv2.moveWindow(IMAGE_WINDOW_NAME, width * 4, 0)
            cv2.resizeWindow(IMAGE_WINDOW_NAME, width * 4, height * 4)
            cv2.setMouseCallback(MAGMA_WINDOW_NAME, onMouse)
            cv2.setMouseCallback(IMAGE_WINDOW_NAME, onMouse)
            initialRescaleAndPositionDone = True
        colormappedFrame = cv2.applyColorMap(colormappedFrame, cv2.COLORMAP_MAGMA)
        if (
            mouseX < 0
            or mouseY < 0
            or mouseX >= thermalData.shape[1]
            or mouseY >= thermalData.shape[0]
        ):
            mouseX = max(0, min(mouseX, thermalData.shape[1] - 1))
            mouseY = max(0, min(mouseY, thermalData.shape[0] - 1))
        textColor = (255, 255, 255)
        # Draw crosshair
        cv2.line(
            colormappedFrame,
            (mouseX - 10, mouseY),
            (mouseX + 10, mouseY),
            textColor,
            1,
        )
        cv2.line(
            colormappedFrame,
            (mouseX, mouseY - 10),
            (mouseX, mouseY + 10),
            textColor,
            1,
        )
        # Draw deg C
        text = f"{thermalData[mouseY, mouseX]:.2f} deg C"
        putTextLeft = mouseX > colormappedFrame.shape[1] / 2
        cv2.putText(
            colormappedFrame,
            text,
            (mouseX - 100 if putTextLeft else mouseX + 10, mouseY - 10),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.5,
            textColor,
            1,
        )

        cv2.imshow(MAGMA_WINDOW_NAME, colormappedFrame)
        cv2.imshow(IMAGE_WINDOW_NAME, inColor.getCvFrame())

        if cv2.waitKey(1) == ord("q"):
            break
