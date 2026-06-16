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
        cfg.setOutputSize(512, 288, ImageManipConfig.ResizeMode.LETTERBOX)
        cfg.setFrameType(ImgFrame.Type.BGR888i)
        cfg.setReusePreviousImage(True)
        node.outputs["config"].send(cfg)
"""

# This Example showcases how to link the following nodes together:
# - An ImageFrame (in this case the passthrough)
# - ImgDetections (the output of the first detection network)
# - Crops from all imgDetections
# - second stage detections on top of those crops
#
#  The final Tree Message should look like this:
#
#      ImgFrame (passthrough)
#         |
#         \/
#     ImgDetections (from first stage detection network)
#     /       |           |             |          \
#    \/       \/          \/            \/           \/
#    Crop 0   Crop 1      Crop 2        Crop 3       Crop 4
#    |         |            |             |             |
#    \/        \/           \/            \/            \/
# Pose Dets  Pose Dets     Pose Dets     Pose Dets      Pose Dets (from second stage detection network running on the crops)


def main() -> None:
    pipeline = dai.Pipeline()
    defaultDevice = pipeline.getDefaultDevice()

    cam = pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_A, sensorFps=15.0)
    firstStageModelDescription = dai.NNModelDescription(
        model="luxonis/yolov8-instance-segmentation-large:coco-640x480",
        platform=defaultDevice.getPlatformAsString(),
    )
    detNN = pipeline.create(dai.node.DetectionNetwork).build(cam, firstStageModelDescription)

    fullResOutput = cam.requestOutput((1920, 1080), dai.ImgFrame.Type.BGR888i)

    filterDetectionsScript = pipeline.create(dai.node.Script)
    filterDetectionsScript.setScript(FILTER_DETECTIONS_SCRIPT)
    detNN.out.link(filterDetectionsScript.inputs["detections"])

    # creates link from ImgFrame --> ImgDetections
    gatherImgFrameImgDetection = pipeline.create(dai.node.GatherData)
    detNN.passthrough.link(gatherImgFrameImgDetection.referenceInput)
    filterDetectionsScript.outputs["filtered"].link(gatherImgFrameImgDetection.collectingInput)

    # Generate crop configs
    cropConfigScript = pipeline.create(dai.node.Script)
    cropConfigScript.inputs["detections"].setMaxSize(20)
    cropConfigScript.setScript(CROP_CONFIG_SCRIPT)
    gatherImgFrameImgDetection.passthroughCollectingInput.link(cropConfigScript.inputs["detections"])

    imageManip = pipeline.create(dai.node.ImageManip)
    imageManip.inputConfig.setMaxSize(50)
    imageManip.inputImage.setMaxSize(20)
    imageManip.inputConfig.setReusePreviousMessage(False)
    imageManip.setMaxOutputFrameSize(288 * 512 * 3)
    cropConfigScript.outputs["config"].link(imageManip.inputConfig)
    fullResOutput.link(imageManip.inputImage)

    gatherCrops = pipeline.create(dai.node.GatherData)
    imageManip.out.link(gatherCrops.collectingInput)
    gatherImgFrameImgDetection.output.link(gatherCrops.referenceInput)
    gatherCrops.setRunOnHost(False)

    # This node will be implicitly created inside each node as a subnode. For demosntration purposes its explicitly created here
    splitCrops = pipeline.create(dai.node.SplitterNode)
    gatherCrops.output.link(splitCrops.input)
    splitCrops.setRunOnHost(False)

    poseModelDescription = dai.NNModelDescription(
        model="luxonis/yolov8-nano-pose-estimation:coco-512x288",
        platform=defaultDevice.getPlatformAsString(),
    )
    poseDetNN = pipeline.create(dai.node.DetectionNetwork).build(
        splitCrops.output,
        dai.NNArchive(dai.getModelFromZoo(poseModelDescription)),
    )

    gatherPoseDets = pipeline.create(dai.node.GatherData)
    gatherCrops.output.link(gatherPoseDets.referenceInput)
    poseDetNN.out.link(gatherPoseDets.collectingInput)
    gatherPoseDets.setRunOnHost(False)

    collectedQ = gatherPoseDets.output.createOutputQueue()

    pipeline.start()

    color = (255, 0, 0)
    keypointColor = (0, 255, 0)
    lastMessageGroupTree = ""
    while pipeline.isRunning():
        collected = collectedQ.get()

        if collected is not None:
            _, lastMessageGroupTree = showMessageGroupTreeIfChanged(
                collected,
                lastMessageGroupTree,
                "Multi-stage Message tree",
            )

        frame = collected.get(0).getCvFrame()  # get the original full frame from the message group

        detections = collected.get(1) if collected is not None else None
        height, width = frame.shape[:2]
        for detection in detections.detections:
            x1 = int(detection.xmin * width)
            y1 = int(detection.ymin * height)
            x2 = int(detection.xmax * width)
            y2 = int(detection.ymax * height)
            cv2.rectangle(frame, (x1, y1), (x2, y2), color, 2)

        cv2.imshow("passthrough", frame)

        for i, detection in enumerate(detections.detections):
            # if you are looking for a specific detection, you can filter out here and only then look at the crop message via getChildren(0,
            # static_cast<uint32_t>(i));

            cropNodeIndices = collected.getChildren(1, i)
            if not cropNodeIndices:
                cv2.destroyWindow(f"crop_{i}")
                continue

            if len(cropNodeIndices) > 1:
                print(
                    f"Expected only one crop message per detection, but found {len(cropNodeIndices)} for detection {i}. Skipping.",
                    file=sys.stderr,
                )
                cv2.destroyWindow(f"crop_{i}")
                continue

            crop = collected.get(cropNodeIndices[0])
            if crop is None:
                cv2.destroyWindow(f"crop_{i}")
                continue

            cropFrame = crop.getCvFrame()
            poseNodeIndices = collected.getChildren(cropNodeIndices[0])
            for poseNodeIndex in poseNodeIndices:
                poseDetections = collected.get(poseNodeIndex)
                if poseDetections is None:
                    continue

                poseDetectionsInCrop = (
                    poseDetections.transformTo(crop.getTransformation())
                    if poseDetections.getTransformation() is not None
                    else poseDetections
                )
                for poseDetection in poseDetectionsInCrop.detections:
                    for keypoint in poseDetection.getKeypoints():
                        keypointPos = (
                            int(keypoint.imageCoordinates.x * cropFrame.shape[1]),
                            int(keypoint.imageCoordinates.y * cropFrame.shape[0]),
                        )
                        cv2.circle(cropFrame, keypointPos, 3, keypointColor, -1)

            className = detection.labelName if detection.labelName else str(detection.label)
            labelPadding = 8
            labelSize, baseline = cv2.getTextSize(className, cv2.FONT_HERSHEY_SIMPLEX, 0.7, 2)
            labelBannerHeight = labelSize[1] + (2 * labelPadding)
            cv2.rectangle(cropFrame, (0, 0), (cropFrame.shape[1], labelBannerHeight), (0, 0, 0), cv2.FILLED)
            cv2.putText(
                cropFrame,
                className,
                (labelPadding, labelBannerHeight - labelPadding),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.7,
                color,
                2,
            )

            cv2.imshow(f"crop_{i}", cropFrame)
            shownCropWindowCount = i + 1


        if cv2.waitKey(1) == ord("q"):
            pipeline.stop()
            break

    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
