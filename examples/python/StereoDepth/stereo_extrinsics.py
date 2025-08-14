#!/usr/bin/env python3

import depthai as dai

def printExtrinsics(extr: dai.Extrinsics):
    print(f"To camera socket: {extr.toCameraSocket}")
    print("Translation x: ", extr.translation.x)
    print("Translation y: ", extr.translation.y)
    print("Translation z: ", extr.translation.z)
    print("Spec translation x: ", extr.specTranslation.x)
    print("Spec translation y: ", extr.specTranslation.y)
    print("Spec translation z: ", extr.specTranslation.z)
    if(extr.toCameraSocket == dai.CameraBoardSocket.AUTO):
        return
    print("Rotation:")
    for i in range(3):
        for j in range(3):
            print(f"{extr.rotationMatrix[i][j]:.6f}", end=' ')
        print()



pipeline = dai.Pipeline()
monoLeft = pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_B)
monoRight = pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_C)
# color = pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_A)
stereo = pipeline.create(dai.node.StereoDepth)

# Linking
monoLeftOut = monoLeft.requestFullResolutionOutput(type=dai.ImgFrame.Type.NV12)
monoRightOut = monoRight.requestFullResolutionOutput(type=dai.ImgFrame.Type.NV12)
monoLeftOut.link(stereo.left)
monoRightOut.link(stereo.right)

stereo.setRectification(True)
stereo.setExtendedDisparity(True)
stereo.setLeftRightCheck(True)

syncedLeftQueue = stereo.syncedLeft.createOutputQueue()
syncedRightQueue = stereo.syncedRight.createOutputQueue()
rectifiedLeftQueue = stereo.rectifiedLeft.createOutputQueue()
rectifiedRightQueue = stereo.rectifiedRight.createOutputQueue()
disparityQueue = stereo.disparity.createOutputQueue()


with pipeline:
    pipeline.start()
    maxDisparity = 1
    while pipeline.isRunning():
        leftSynced = syncedLeftQueue.get()
        rightSynced = syncedRightQueue.get()
        leftRectified = rectifiedLeftQueue.get()
        rightRectified = rectifiedRightQueue.get()
        disparity = disparityQueue.get()

        assert isinstance(leftSynced, dai.ImgFrame)
        assert isinstance(rightSynced, dai.ImgFrame)
        assert isinstance(disparity, dai.ImgFrame)
        print("Extrinsics")
        print("Left Synced:")
        printExtrinsics(leftSynced.getTransformation().getExtrinsics())
        print("Left Rectified:")
        printExtrinsics(leftRectified.getTransformation().getExtrinsics())
        print("Right Synced:")
        printExtrinsics(rightSynced.getTransformation().getExtrinsics())
        print("Right Rectified:")
        printExtrinsics(rightRectified.getTransformation().getExtrinsics())
        print("Disparity:")
        printExtrinsics(disparityQueue.get().getTransformation().getExtrinsics())


        
