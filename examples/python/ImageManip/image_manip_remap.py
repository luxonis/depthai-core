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

with dai.Pipeline() as pipeline:
    cam = pipeline.create(dai.node.Camera).build()
    camOut = cam.requestOutput((640, 400), dai.ImgFrame.Type.BGR888i, fps = 30.0)
    manip1 = pipeline.create(dai.node.ImageManip)
    manip2 = pipeline.create(dai.node.ImageManip)

    camOut.link(manip1.inputImage)
    manip1.out.link(manip2.inputImage)

    manip1.initialConfig.addRotateDeg(90)
    manip1.initialConfig.setOutputSize(200, 320)

    manip2.initialConfig.addRotateDeg(90)
    manip2.initialConfig.setOutputSize(320, 200)
    manip2.setRunOnHost(True)

    outQcam = camOut.createOutputQueue()
    outQ1 = manip1.out.createOutputQueue()
    outQ2 = manip2.out.createOutputQueue()

    pipeline.start()

    while True:
        camFrame: dai.ImgFrame = outQcam.get()
        manip1Frame: dai.ImgFrame = outQ1.get()
        manip2Frame: dai.ImgFrame = outQ2.get()

        camCv = camFrame.getCvFrame()
        manip1Cv = manip1Frame.getCvFrame()
        manip2Cv = manip2Frame.getCvFrame()

        rect2 = dai.RotatedRect(dai.Rect(dai.Point2f(100, 100), dai.Point2f(200, 150)), 0)
        rect1 = manip2Frame.getTransformation().remapRectTo(manip1Frame.getTransformation(), rect2)
        rectcam = manip1Frame.getTransformation().remapRectTo(camFrame.getTransformation(), rect1)

        draw_rotated_rectangle(manip2Cv, (rect2.center.x, rect2.center.y), (rect2.size.width, rect2.size.height), rect2.angle, (255, 0, 0))
        draw_rotated_rectangle(manip1Cv, (rect1.center.x, rect1.center.y), (rect1.size.width, rect1.size.height), rect1.angle, (255, 0, 0))
        draw_rotated_rectangle(camCv, (rectcam.center.x, rectcam.center.y), (rectcam.size.width, rectcam.size.height), rectcam.angle, (255, 0, 0))

        cv2.imshow("cam", camCv)
        cv2.imshow("manip1", manip1Cv)
        cv2.imshow("manip2", manip2Cv)
        if cv2.waitKey(1) == ord('q'):
            break
