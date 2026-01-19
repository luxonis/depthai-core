#!/usr/bin/env python3

import cv2
import depthai as dai
import numpy as np
import time


# Callback functions
def uploadSuccessCallback(sendSnapResult):
    assert (sendSnapResult.uploadStatus == dai.SendSnapCallbackStatus.SUCCESS)
    print(f"Successfully uploaded Snap: ({sendSnapResult.snapName}, {sendSnapResult.snapTimestamp}, {sendSnapResult.snapHubID}) to the hub.")

def uploadFailureCallback(sendSnapResult):
    assert (sendSnapResult.uploadStatus != dai.SendSnapCallbackStatus.SUCCESS)
    print(f"Upload of Snap: ({sendSnapResult.snapName}, {sendSnapResult.snapTimestamp}, {sendSnapResult.snapLocalID}) to the hub has failed.")

    status = sendSnapResult.uploadStatus
    if status == dai.SendSnapCallbackStatus.FILE_BATCH_PREPARATION_FAILED:
        print("File batch preparation failed!")
    elif status == dai.SendSnapCallbackStatus.GROUP_CONTAINS_REJECTED_FILES:
        print("Snap's file group contains rejected files!")
    elif status == dai.SendSnapCallbackStatus.FILE_UPLOAD_FAILED:
        print("File upload was unsuccessful!")
    elif status == dai.SendSnapCallbackStatus.SEND_EVENT_FAILED:
        print("Snap could not been sent to the hub, following successful file upload!")

# Create pipeline
with dai.Pipeline() as pipeline:
    # Set your Hub team's api-key using the environment variable DEPTHAI_HUB_API_KEY. Or use the EventsManager setToken() method.
    eventMan = dai.EventsManager()

    cameraNode = pipeline.create(dai.node.Camera).build()
    detectionNetwork = pipeline.create(dai.node.DetectionNetwork).build(cameraNode, dai.NNModelDescription("yolov6-nano"))
    labelMap = detectionNetwork.getClasses()

    # Create output queues
    qRgb = detectionNetwork.passthrough.createOutputQueue()
    qDet = detectionNetwork.out.createOutputQueue()

    pipeline.start()


    # nn data, being the bounding box locations, are in <0..1> range - they need to be normalized with frame width/height
    def frameNorm(frame, bbox):
        normVals = np.full(len(bbox), frame.shape[0])
        normVals[::2] = frame.shape[1]
        return (np.clip(np.array(bbox), 0, 1) * normVals).astype(int)


    while pipeline.isRunning():
        if cv2.waitKey(1) != -1:
            pipeline.stop()
            break

        inRgb: dai.ImgFrame = qRgb.get()
        inDet: dai.ImgDetections = qDet.get()
        if inRgb is None or inDet is None:
            continue

        # Display the video stream and detections
        color = (255, 0, 0)
        frame = inRgb.getCvFrame()
        if frame is not None:
            for detection in inDet.detections:
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
            cv2.imshow("rgb", frame)

        # Trigger sendSnap()
        if cv2.waitKey(1) == ord("s"):
            localSnapID = eventMan.sendSnap("ImageDetection", None, inRgb, inDet, ["EventsExample", "Python"], {"key_0" : "value_0", "key_1" : "value_1"}, 
                              uploadSuccessCallback, uploadFailureCallback)
            if localSnapID != None:
                print(f"Snap with a localID: {localSnapID} has been successfully added to the EventsManager")
            else:
                print("Snap was not successfully added to the EventsManager")