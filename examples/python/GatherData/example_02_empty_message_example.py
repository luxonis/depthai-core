#!/usr/bin/env python3

import sys

import cv2
import depthai as dai
import numpy as np

from examples.python.GatherData.message_group_visualizer import showMessageGroupTreeIfChanged

FILTER_DETECTIONS_SCRIPT = r"""
from depthai import ImgDetection, ImgDetections

while True:
    src = node.inputs["detections"].get()
    if src is None:
        break

    # Send empty detections to simualte no detections found.
    dst = ImgDetections()
    dst.setSequenceNum(src.getSequenceNum())
    dst.setTimestamp(src.getTimestamp())
    dst.setTimestampDevice(src.getTimestampDevice())
    dst.setTransformation(src.getTransformation())

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


#This Example showcases if no detections are foun in the first frame. This means that each GatherData node for loop will iterate over 0 loops, adding just an EmptyMessage in that layer.
# The final Tree Message should look like this:
#  *
#                ImgFrame (passthrough)
#                        |
#                        \/
#    ImgDetections (from first stage detection network)
#                        |
#                        \/
#                    EmptyMessage
#                        |
#                        \/
#                 EmptyMessage


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

    while pipeline.isRunning():
        collected = collectedQ.get()
        lastMessageGroupTree = ""

        if collected is not None:
            _, lastMessageGroupTree = showMessageGroupTreeIfChanged(
                collected,
                lastMessageGroupTree,
                "Multi-stage Message tree",
            )


        frame = collected.get(0).getCvFrame()  # get the original full frame from the message group
        assert isinstance(frame, np.ndarray)
        
        frame_children = collected.getChildren(0)

        assert len(frame_children) == 1
        imgDetectionsMsgIndex = frame_children[0]
        imgDetectionsMsg = collected.get(imgDetectionsMsgIndex)
        assert isinstance(imgDetectionsMsg, dai.ImgDetections)
        assert len(imgDetectionsMsg.detections) == 0
        
        imgDetectionsChildren = collected.getChildren(imgDetectionsMsgIndex)
        
        firstEmptyMessageIndex = imgDetectionsChildren[0]
        assert len(imgDetectionsChildren) == 1
        firstEmptyMessage = collected.get(firstEmptyMessageIndex)
        assert isinstance(firstEmptyMessage, dai.EmptyMessage)
        
        secondEmptyMessageIndex = collected.getChildren(firstEmptyMessageIndex)[0]
        secondEmptyMessage = collected.get(secondEmptyMessageIndex)
        assert isinstance(secondEmptyMessage, dai.EmptyMessage)
        
        print("All checks passed. The GatherData tree is as expected with an EmptyMessage after the ImgDetections message.")


if __name__ == "__main__":
    main()
