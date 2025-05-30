import depthai as dai
import cv2

pipeline = dai.Pipeline()

manip_input = pipeline.create(dai.node.ImageManip)
manip_input.initialConfig.setFrameType(dai.ImgFrame.Type.BGR888p)
inputQueue = manip_input.inputImage.createInputQueue()

manip_ops = [
    # Resize operations. If aspect ratio isn't the same, the image will be stretched/cropped/letterboxed (depending on resize mode)
    # Docs here: https://docs.luxonis.com/software/depthai/resolution-techniques/
    ('resize_stretch', lambda conf: conf.setOutputSize(256, 200, dai.ImageManipConfig.ResizeMode.STRETCH)),
    ('resize_letterbox', lambda conf: conf.setOutputSize(256, 200, dai.ImageManipConfig.ResizeMode.LETTERBOX)),
    ('resize_center_crop', lambda conf: conf.setOutputSize(256, 200, dai.ImageManipConfig.ResizeMode.CENTER_CROP)),
    # Crop the image topLeft (10,40) to bottomRight (310,110)
    ('crop', lambda conf: conf.addCrop(x=50, y=50, w=150, h=200)),
    # Flip the frame vertically/horizontally
    ('flip_vertical', lambda conf: conf.addFlipVertical()),
    ('flip_horizontal', lambda conf: conf.addFlipHorizontal()),
    # Scale the image by 0.7x in x and 0.5x in y
    ('scale', lambda conf: conf.addScale(0.7, 0.5)),
    # Rotate. If center isn't specified, it will rotate around center (0.5, 0.5)
    ('rotate_90_deg', lambda conf: conf.addRotateDeg(90)),
    ('rotate_90_deg_center', lambda conf: conf.addRotateDeg(90, center=dai.Point2f(0.2, 0.3)).setOutputCenter(False)),
    ('transform_affine', lambda conf: conf.addTransformAffine( # Shearing
        [1, 0.5,
         0.2, 1])),
    ('transform_perspective', lambda conf: conf.addTransformPerspective(
        [1.0, 0.2, 0.0,  # First row
        0.1, 1.0, 0.0,  # Second row
        0.001, 0.002, 1.0])),  # Third row
    ('frame_type', lambda conf: conf.setFrameType(dai.ImgFrame.Type.RAW8)), # to Grayscale
]

# Dynamically create ImageManip nodes, apply configurations, and set up queues
queues = {}
for name, config in manip_ops:
    print(name, config)
    manip = pipeline.create(dai.node.ImageManip)
    config(manip.initialConfig)
    manip_input.out.link(manip.inputImage)
    queues[name] = manip.out.createOutputQueue(maxSize=4, blocking=False)


imgFrame = dai.ImgFrame()

input_frame = cv2.imread('../models/lenna.png') # 512x512
# Send 256x256 image to the device
imgFrame.setCvFrame(cv2.pyrDown(input_frame), dai.ImgFrame.Type.BGR888i)
inputQueue.send(imgFrame)

cv2.imshow('input_image', input_frame)


pipeline.start()

for name, queue in queues.items():
    inFrame = queue.get()
    cv2.imshow(name, inFrame.getCvFrame())

key = cv2.waitKey(0)
