import depthai as dai

import cv2

RUN_ON_HOST = True

modelDescription = dai.NNModelDescription(modelSlug="mediapipe-face-landmarker", modelVersionSlug="192x192", platform="RVC2")
archivePath = dai.getModelFromZoo(modelDescription, useCached=True)
nnArchive = dai.NNArchive(archivePath)

with dai.Pipeline() as pipeline:

    print("Creating pipeline...")
    cam = pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_A)
    full = cam.requestOutput((720, 720), dai.ImgFrame.Type.BGR888p)

    manip = pipeline.create(dai.node.ImageManip)
    manip.initialConfig.setResize(192, 192)
    full.link(manip.inputImage)

    nn = pipeline.create(dai.node.NeuralNetwork).build(
        input=manip.out,
        nnArchive=nnArchive
    )

    parser = pipeline.create(dai.node.KeypointsParser)
    parser.setNumKeypoints(468)
    parser.setScaleFactor(192)
    parser.setRunOnHost(RUN_ON_HOST)
    nn.out.link(parser.input)

    video_q = full.createOutputQueue()
    keypoints_q = parser.out.createOutputQueue()

    pipeline.start()

    while pipeline.isRunning():
        frame = video_q.get().getCvFrame()
        keypoints: dai.Keypoints = keypoints_q.get()

        for keypoint in keypoints.getKeypoints():
            x, y = keypoint.x, keypoint.y
            x, y = int(x * frame.shape[1]), int(y * frame.shape[0])
            frame = cv2.circle(frame, (int(x), int(y)), 2, (0, 255, 0), -1)

        cv2.imshow("Frame", frame)
        if cv2.waitKey(1) == ord("q"):
            break
