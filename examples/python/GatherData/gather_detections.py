#!/usr/bin/env python3

import sys

import cv2
import depthai as dai

from message_group_visualizer import showMessageGroupTreeIfChanged


FILTER_DETECTIONS_SCRIPT = r"""
from depthai import ImgDetection, ImgDetections

kMaxDetections = 7

while True:
    src = node.inputs["detections"].get()
    if src is None:
        break

    # Build a fresh message so truncating detections does not leave a stale
    # instance-segmentation payload attached to the output message.
    dst = ImgDetections()
    dst.setSequenceNum(src.getSequenceNum())
    dst.setTimestamp(src.getTimestamp())
    dst.setTimestampDevice(src.getTimestampDevice())
    dst.setTransformation(src.getTransformation())

    filtered_detections = []
    for det in src.detections[:kMaxDetections]:
        copied = ImgDetection()
        copied.label = det.label
        copied.labelName = det.labelName
        copied.confidence = det.confidence

        try:
            copied.setBoundingBox(det.getBoundingBox())
        except Exception:
            pass

        keypoints = det.getKeypoints()
        if len(keypoints) > 0:
            copied.setKeypoints(keypoints)
            edges = det.getEdges()
            if len(edges) > 0:
                copied.setEdges(edges)

        filtered_detections.append(copied)

    dst.detections = filtered_detections
    node.outputs["filtered"].send(dst)
"""


CROP_CONFIG_SCRIPT = r"""
from depthai import ImageManipConfig, ImgFrame, Rect, RotatedRect

while True:
    detections_msg = node.inputs["detections"].get()
    if detections_msg is None:
        break

    clear_cfg = ImageManipConfig()
    clear_cfg.setSkipCurrentImage(True)
    node.outputs["config"].send(clear_cfg)

    for index, detection in enumerate(detections_msg.detections):
        outer_bbox = detection.getBoundingBox().getOuterRect()
        x1 = max(0.0, min(outer_bbox[0], 0.9999))
        y1 = max(0.0, min(outer_bbox[1], 0.9999))
        x2 = max(0.0, min(outer_bbox[2], 0.9999))
        y2 = max(0.0, min(outer_bbox[3], 0.9999))

        cfg = ImageManipConfig()
        cfg.addCropRotatedRect(RotatedRect(Rect(x1, y1, x2 - x1, y2 - y1, True)), True)
        cfg.setOutputSize(320, 240, ImageManipConfig.ResizeMode.LETTERBOX)
        cfg.setFrameType(ImgFrame.Type.BGR888i)
        cfg.setReusePreviousImage(True)
        node.outputs["config"].send(cfg)
"""


def main() -> None:
    pipeline = dai.Pipeline()

    cam = pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_A)
    detNN = pipeline.create(dai.node.DetectionNetwork).build(
        cam,
        dai.NNModelDescription("luxonis/yolov8-instance-segmentation-large:coco-640x480"),
    )

    fullResOutput = cam.requestOutput((1280, 800))

    filterDetectionsScript = pipeline.create(dai.node.Script)
    filterDetectionsScript.setScript(FILTER_DETECTIONS_SCRIPT)
    detNN.out.link(filterDetectionsScript.inputs["detections"])

    # Generate crop configs
    cropConfigScript = pipeline.create(dai.node.Script)
    cropConfigScript.inputs["detections"].setMaxSize(10)
    cropConfigScript.setScript(CROP_CONFIG_SCRIPT)
    filterDetectionsScript.outputs["filtered"].link(cropConfigScript.inputs["detections"])

    imageManip = pipeline.create(dai.node.ImageManip)
    imageManip.inputConfig.setMaxSize(50)
    imageManip.inputImage.setMaxSize(10)
    imageManip.inputConfig.setReusePreviousMessage(False)

    imageManip.setMaxOutputFrameSize(320 * 240 * 3)
    cropConfigScript.outputs["config"].link(imageManip.inputConfig)
    fullResOutput.link(imageManip.inputImage)
    # end Create crops

    gatherData = pipeline.create(dai.node.GatherData)
    # gatherData->setRunOnHost(true);
    filterDetectionsScript.outputs["filtered"].link(gatherData.referenceInput)
    imageManip.out.link(gatherData.collectingInput)

    collectedQ = gatherData.output.createOutputQueue()
    passthroughQ = detNN.passthrough.createOutputQueue()

    pipeline.start()

    color = (255, 0, 0)
    lastMessageGroupTree = ""
    while pipeline.isRunning():
        collected = collectedQ.get()
        print("Got a new collected message group")
        passthrough = passthroughQ.get()
        print("Got a new passthrough message")

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
