#!/usr/bin/env python3

import sys
import time

import cv2
import depthai as dai

from message_group_visualizer import showMessageGroupTreeIfChanged


class CropConfigsCreator(dai.node.ThreadedHostNode):
    def __init__(self) -> None:
        super().__init__()
        self.configOutput = self.createOutput("config_output")
        self.configOutput.setPossibleDatatypes([(dai.DatatypeEnum.ImageManipConfig, True)])
        self.detectionsInput = self.createInput("detections_input", waitForMessage=False)
        self.detectionsInput.setPossibleDatatypes([(dai.DatatypeEnum.ImgDetections, True)])

    def run(self) -> None:
        while self.mainLoop():
            detectionsInputMessage = self.detectionsInput.get()
            if detectionsInputMessage is None:
                continue

            detections = detectionsInputMessage.detections

            clear_cfg = dai.ImageManipConfig()
            clear_cfg.setSkipCurrentImage(True)
            send_status = self.configOutput.trySend(clear_cfg)

            for i in range(len(detections)):
                cfg = dai.ImageManipConfig()

                detection = detections[i]

                bbox = detection.getBoundingBox()
                outerBbox = bbox.getOuterRect()
                x1 = max(0.0, min(outerBbox[0], 0.9999))
                y1 = max(0.0, min(outerBbox[1], 0.9999))
                x2 = max(0.0, min(outerBbox[2], 0.9999))
                y2 = max(0.0, min(outerBbox[3], 0.9999))
                bbox = dai.RotatedRect(dai.Rect(x1, y1, x2 - x1, y2 - y1, True))

                cfg.addCropRotatedRect(bbox, True)
                cfg.setOutputSize(300, 400, dai.ImageManipConfig.ResizeMode.CENTER_CROP)
                if i == 0:
                    cfg.setReusePreviousImage(False)
                else:
                    cfg.setReusePreviousImage(True)

                cfgMessage = cfg
                sendStatus = False
                attempts = 0
                while not sendStatus and attempts < 100 and self.mainLoop():
                    sendStatus = self.configOutput.trySend(cfgMessage)
                    if not sendStatus:
                        attempts += 1
                        time.sleep(0.001)


def main() -> None:
    pipeline = dai.Pipeline()

    cam = pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_A)
    detNN = pipeline.create(dai.node.DetectionNetwork).build(
        cam,
        dai.NNModelDescription("luxonis/yolov8-instance-segmentation-large:coco-640x480"),
    )

    fullResOutput = cam.requestOutput((1280, 800))

    # Create crops
    cropConfigsCreator = pipeline.create(CropConfigsCreator)
    detNN.out.link(cropConfigsCreator.detectionsInput)

    imageManip = pipeline.create(dai.node.ImageManip)
    imageManip.inputConfig.setReusePreviousMessage(False)

    imageManip.setMaxOutputFrameSize(300 * 480 * 3)
    cropConfigsCreator.configOutput.link(imageManip.inputConfig)
    fullResOutput.link(imageManip.inputImage)
    # end Create crops

    gatherData = pipeline.create(dai.node.GatherData)
    # gatherData->setRunOnHost(true);
    detNN.out.link(gatherData.referenceInput)
    imageManip.out.link(gatherData.collectingInput)

    collectedQ = gatherData.output.createOutputQueue()
    passthroughQ = detNN.passthrough.createOutputQueue()

    pipeline.start()

    color = (255, 0, 0)
    lastMessageGroupTree = ""
    while pipeline.isRunning():
        collected = collectedQ.get()
        passthrough = passthroughQ.get()

        if collected is not None:
            _, lastMessageGroupTree = showMessageGroupTreeIfChanged(collected, lastMessageGroupTree, "GatherData tree")

        detections = collected.get(0) if collected is not None else None

        if collected is None or detections is None or passthrough is None:
            continue

        # std::cout << "Got " << detections->detections.size() << " detections)" << std::endl;
        # std::cout << "Message group has " << collected->getNumMessages() << " messages." << std::endl;

        frame = passthrough.getCvFrame()
        height, width = frame.shape[:2]
        for detection in detections.detections:
            x1 = int(detection.xmin * width)
            y1 = int(detection.ymin * height)
            x2 = int(detection.xmax * width)
            y2 = int(detection.ymax * height)
            cv2.rectangle(frame, (x1, y1), (x2, y2), color, 2)

        cv2.imshow("passthrough", frame)

        for i in range(len(detections.detections)):
            detection = detections.detections[i]

            # if you are looking for a specific detection, you can filter out here and only then look at the crop message via getChildren(0,
            # static_cast<uint32_t>(i));

            cropNodeIndices = collected.getChildren(0, i)
            if not cropNodeIndices:
                continue

            if len(cropNodeIndices) > 1:
                print(
                    f"Expected only one crop message per detection, but found {len(cropNodeIndices)} for detection {i}. Skipping.",
                    file=sys.stderr,
                )
                continue

            crop = collected.get(cropNodeIndices[0])
            if crop is None:
                continue

            cropFrame = crop.getCvFrame()
            className = str(detection.label) if not detection.labelName else detection.labelName
            detWidth = int(detection.getWidth() * width)
            detHeight = int(detection.getHeight() * height)

            print(f"Detection {i}: class: {className}, size: {detWidth}x{detHeight}")

            cv2.putText(cropFrame, "class: " + className, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)
            cv2.putText(
                cropFrame,
                "size: " + str(detWidth) + "x" + str(detHeight),
                (10, 60),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.6,
                color,
                2,
            )
            cv2.imshow("crop_" + str(i), cropFrame)

        if cv2.waitKey(1) == ord("q"):
            pipeline.stop()
            break

    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
