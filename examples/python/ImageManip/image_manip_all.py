import depthai as dai
import cv2

pipeline = dai.Pipeline()

manip_input = pipeline.create(dai.node.ImageManipV2)
manip_input.initialConfig.setFrameType(dai.ImgFrame.Type.BGR888p)
inputQueue = manip_input.inputImage.createInputQueue()

manip_ops = [
    # Resize operations. If aspect ratio isn't the same, the image will be stretched/cropped/letterboxed (depending on resize mode)
    # Docs here: https://docs.luxonis.com/software/depthai/resolution-techniques/
    ('resize_stretch', lambda conf: conf.setOutputSize(240, 240, dai.ImageManipConfigV2.ResizeMode.STRETCH)),
    ('resize_letterbox', lambda conf: conf.setOutputSize(240, 240, dai.ImageManipConfigV2.ResizeMode.LETTERBOX)),
    ('resize_center_crop', lambda conf: conf.setOutputSize(240, 240, dai.ImageManipConfigV2.ResizeMode.CENTER_CROP)),
    # Crop the image topLeft (50,100) to bottomRight (500,500)
    ('crop', lambda conf: conf.addCrop(50, 100, 100, 140)),
    # Flip the frame vertically/horizontally
    ('flip_vertical', lambda conf: conf.addFlipVertical()),
    ('flip_horizontal', lambda conf: conf.addFlipHorizontal()),
    # Scale the image by 0.7x in x and 0.5x in y
    ('scale', lambda conf: conf.addScale(0.7, 0.5)),
    # Rotate. If center isn't specified, it will rotate around center (0.5, 0.5)
    ('rotate_90_deg', lambda conf: conf.addRotateDeg(90)),
    ('rotate_90_deg_center', lambda conf: conf.addRotateDeg(90, center=dai.Point2f(0.3, 0.3))),
]

# Dynamically create ImageManipV2 nodes, apply configurations, and set up queues
queues = {}
for name, config in manip_ops:
    print(name, config)
    manip = pipeline.create(dai.node.ImageManipV2)
    config(manip.initialConfig)
    manip_input.out.link(manip.inputImage)
    queues[name] = manip.out.createOutputQueue(maxSize=4, blocking=False)

imgFrame = dai.ImgFrame()
input_frame = cv2.imread('luxonis-vga.jpeg')
imgFrame.setFrame(cv2.pyrDown(input_frame))
imgFrame.setWidth(320)
imgFrame.setHeight(240)
imgFrame.setType(dai.ImgFrame.Type.BGR888i)
inputQueue.send(imgFrame)

cv2.imshow('input_image', input_frame)

pipeline.start()
while True:
    for name, queue in queues.items():
        inFrame = queue.get()
        if inFrame is not None:
            cv2.imshow(name, inFrame.getCvFrame())

    key = cv2.waitKey(0)
    if key == ord('q'):
        break