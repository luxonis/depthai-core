import depthai as dai
import cv2
import numpy as np

def draw_rotated_rectangle(frame, center, size, angle, color, thickness=2):
    """
    Draws a rotated rectangle on the given frame.

    Args:
        frame (numpy.ndarray): The image/frame to draw on.
        center (tuple): The (x, y) coordinates of the rectangle's center.
        size (tuple): The (width, height) of the rectangle.
        angle (float): The rotation angle of the rectangle in degrees (counter-clockwise).
        color (tuple): The color of the rectangle in BGR format (e.g., (0, 255, 0) for green).
        thickness (int): The thickness of the rectangle edges. Default is 2.
    """
    # Create a rotated rectangle
    rect = ((center[0], center[1]), (size[0], size[1]), angle)

    # Get the four vertices of the rotated rectangle
    box = cv2.boxPoints(rect)
    box = np.intp(box)  # Convert to integer coordinates

    # Draw the rectangle on the frame
    cv2.polylines(frame, [box], isClosed=True, color=color, thickness=thickness)

def processDepthFrame(depthFrame):
    depth_downscaled = depthFrame[::4]
    if np.all(depth_downscaled == 0):
        min_depth = 0
    else:
        min_depth = np.percentile(depth_downscaled[depth_downscaled != 0], 1)
    max_depth = np.percentile(depth_downscaled, 99)
    depthFrameColor = np.interp(depthFrame, (min_depth, max_depth), (0, 255)).astype(np.uint8)
    return cv2.applyColorMap(depthFrameColor, cv2.COLORMAP_HOT)

with dai.Pipeline() as pipeline:
    color = pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_A)
    monoLeft = pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_B)
    monoRight = pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_C)
    stereo = pipeline.create(dai.node.StereoDepth)

    stereo.setDefaultProfilePreset(dai.node.StereoDepth.PresetMode.DEFAULT)
    # stereo.setDepthAlign(dai.CameraBoardSocket.CAM_A)
    # stereo.setOutputSize(640, 400)

    colorCamOut = color.requestOutput((640, 480))

    monoLeftOut = monoLeft.requestOutput((640, 480))
    monoRightOut = monoRight.requestOutput((640, 480))

    monoLeftOut.link(stereo.left)
    monoRightOut.link(stereo.right)

    colorOut = colorCamOut.createOutputQueue()
    rightOut = monoRightOut.createOutputQueue()
    stereoOut = stereo.depth.createOutputQueue()

    pipeline.start()
    while pipeline.isRunning():
        colorFrame = colorOut.get()
        stereoFrame = stereoOut.get()

        assert colorFrame.validateTransformations()
        assert stereoFrame.validateTransformations()

        clr = colorFrame.getCvFrame()
        depth = processDepthFrame(stereoFrame.getCvFrame())

        rect = dai.RotatedRect(dai.Point2f(300, 200), dai.Size2f(200, 100), 10)
        remappedRect = colorFrame.getTransformation().remapRectTo(stereoFrame.getTransformation(), rect)

        print(f"Original rect x: {rect.center.x} y: {rect.center.y} width: {rect.size.width} height: {rect.size.height} angle: {rect.angle}")
        print(f"Remapped rect x: {remappedRect.center.x} y: {remappedRect.center.y} width: {remappedRect.size.width} height: {remappedRect.size.height} angle: {remappedRect.angle}")

        draw_rotated_rectangle(clr, (rect.center.x, rect.center.y), (rect.size.width, rect.size.height), rect.angle, (255, 0, 0))
        draw_rotated_rectangle(depth, (remappedRect.center.x, remappedRect.center.y), (remappedRect.size.width, remappedRect.size.height), remappedRect.angle, (255, 0, 0))

        cv2.imshow("color", clr)
        cv2.imshow("depth", depth)

        if cv2.waitKey(1) == ord('q'):
            break
    pipeline.stop()
