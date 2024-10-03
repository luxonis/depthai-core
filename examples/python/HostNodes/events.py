
#!/usr/bin/env python3

import cv2
import depthai as dai
import numpy as np
import time

# Get argument first
modelDescription = dai.NNModelDescription(modelSlug="yolov6-nano", platform="RVC2")
archivePath = dai.getModelFromZoo(modelDescription, useCached=True)

# Create pipeline
with dai.Pipeline() as pipeline:
    # Define sources and outputs
    camRgb = pipeline.create(dai.node.ColorCamera)
    # Properties
    camRgb.setResolution(dai.ColorCameraProperties.SensorResolution.THE_1080_P)
    camRgb.setInterleaved(False)
    camRgb.setColorOrder(dai.ColorCameraProperties.ColorOrder.BGR)
    camRgb.setFps(15)
    nnArchive = dai.NNArchive(archivePath)
    h, w = nnArchive.getConfig().getConfigV1().model.inputs[0].shape[-2:]
    camRgb.setPreviewSize(w, h)
    detectionNetwork = pipeline.create(dai.node.DetectionNetwork).build(
        camRgb.preview, nnArchive
    )
    detectionNetwork.setNumInferenceThreads(2)

    # If needed, you can set the NNArchive by yourself
    # detectionNetwork.setNNArchive(nnArchive)

    # If nnArchive.getModelType() == dai.ModelType.SUPERBLOB
    # you can specify the number of shaves
    # detectionNetwork.setNNArchive(nnArchive, numShaves=9)
    # When ^^^ is used and the archive type is not SUPERBLOB, an exception will be thrown

    qRgb = detectionNetwork.passthrough.createOutputQueue()
    qDet = detectionNetwork.out.createOutputQueue()

    labelMap = detectionNetwork.getClasses()
    eventMan = dai.EventsManager()
    eventMan.setLogResponse(True)
    eventMan.setUrl("http://0.0.0.0:80/post")
	# eventsManager->sendSnap("test3", {{"key3", "value3"}}, {"tag5", "tag6"});

    eventMan.sendSnap("test1", {"key1": "key2"}, ["tag1"])
    eventMan.sendSnap("test2", {"key3": "key4"}, ["tag2"])
    eventMan.sendSnap("test3", {"key5": "key6"}, ["tag3"])
    eventMan.sendSnap("test4", {"key7": "key8"}, ["tag4"])
    eventMan.sendSnap("test5", {"key9": "key10"}, ["tag5"])
    eventMan.sendSnap("test6", {"key11": "key12"}, ["tag6"])
    eventMan.sendSnap("test7", {"key13": "key14"}, ["tag7"])
    eventMan.sendSnap("test8", {"key15": "key16"}, ["tag8"])
    eventMan.sendSnap("test9", {"key17": "key18"}, ["tag9"])
    time.sleep(2)
    fileData = dai.FileData()
    fileData.data = b'Hello, world!'
    fileData.fileName = "hello.txt"
    fileData.mimeType = "text/plain"
    eventMan.sendSnap("test10", {"key19": "key20"}, ["tag10"], [fileData])
    fileData2 = dai.FileData()
    fileData2.fileUrl = "/test.txt"
    eventMan.sendSnap("test11", {"key21": "key22"}, ["tag11"], [fileData2])
    eventMan.sendSnap("test12", {"key23": "key24"}, ["tag12"], [fileData, fileData2])
    pipeline.start()

    frame = None
    detections = []
    startTime = time.monotonic()
    counter = 0
    color2 = (255, 255, 255)

    # nn data, being the bounding box locations, are in <0..1> range - they need to be normalized with frame width/height
    def frameNorm(frame, bbox):
        normVals = np.full(len(bbox), frame.shape[0])
        normVals[::2] = frame.shape[1]
        return (np.clip(np.array(bbox), 0, 1) * normVals).astype(int)

    def displayFrame(name, frame):
        color = (255, 0, 0)
        for detection in detections:
            bbox = frameNorm(
                frame,
                (detection.xmin, detection.ymin, detection.xmax, detection.ymax),
            )
            cv2.putText(
                frame,
                labelMap[detection.label],
                (bbox[0] + 10, bbox[1] + 20),
                cv2.FONT_HERSHEY_TRIPLEX,
                0.5,
                255,
            )
            cv2.putText(
                frame,
                f"{int(detection.confidence * 100)}%",
                (bbox[0] + 10, bbox[1] + 40),
                cv2.FONT_HERSHEY_TRIPLEX,
                0.5,
                255,
            )
            cv2.rectangle(frame, (bbox[0], bbox[1]), (bbox[2], bbox[3]), color, 2)
        # Show the frame
        cv2.imshow(name, frame)

    eventSent = False
    while pipeline.isRunning():
        inRgb: dai.ImgFrame = qRgb.get()
        inDet: dai.ImgDetections = qDet.get()
        if inRgb is not None:
            frame = inRgb.getCvFrame()
            if not eventSent:
                # eventMan.sendSnap("test5", {"key9": "key10"}, ["tag5"], [], inRgb)
                eventSent = True
            cv2.putText(
                frame,
                "NN fps: {:.2f}".format(counter / (time.monotonic() - startTime)),
                (2, frame.shape[0] - 4),
                cv2.FONT_HERSHEY_TRIPLEX,
                0.4,
                color2,
            )

        if inDet is not None:
            detections = inDet.detections
            counter += 1

        if frame is not None:
            displayFrame("rgb", frame)

        if cv2.waitKey(1) == ord("q"):
            pipeline.stop()
            break
