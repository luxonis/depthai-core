import numpy as np
import depthai as dai
import cv2

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
    box = np.int0(box)  # Convert to integer coordinates

    # Draw the rectangle on the frame
    cv2.polylines(frame, [box], isClosed=True, color=color, thickness=thickness)

with dai.Pipeline() as pipeline:
    monoLeft = pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_B)
    monoRight = pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_C)
    align = pipeline.create(dai.node.ImageAlign)

    monoLeftOut = monoLeft.requestOutput((640, 480))
    monoRightOut = monoRight.requestOutput((640, 480))

    monoLeftOut.link(align.input)
    monoRightOut.link(align.inputAlignTo)

    rightOut = monoRightOut.createOutputQueue()
    alignOut = align.outputAligned.createOutputQueue()

    pipeline.start()
    while pipeline.isRunning():
        rightFrame = rightOut.get()
        alignedFrame = alignOut.get()

        print(alignedFrame)

        assert rightFrame.validateTransformations()
        assert alignedFrame.validateTransformations()

        right = rightFrame.getCvFrame()
        depth = alignedFrame.getCvFrame()

        rect = dai.RotatedRect(dai.Point2f(200, 100), dai.Size2f(200, 100), 10)
        remappedRect = rightFrame.getTransformation().remapRectTo(alignedFrame.getTransformation(), rect)

        draw_rotated_rectangle(right, (rect.center.x, rect.center.y), (rect.size.width, rect.size.height), rect.angle, (255, 0, 0))
        draw_rotated_rectangle(depth, (remappedRect.center.x, remappedRect.center.y), (remappedRect.size.width, remappedRect.size.height), remappedRect.angle, (255, 0, 0))

        cv2.imshow("right", right)
        cv2.imshow("aligned", depth)

        if cv2.waitKey(1) == ord('q'):
            break
    pipeline.stop()
