import depthai as dai
import numpy as np
import cv2

with dai.Pipeline() as pipeline:
    # Pipeline debugging is disabled by default.
    # You can also enable it by setting the DEPTHAI_PIPELINE_DEBUGGING environment variable to '1'
    pipeline.enablePipelineDebugging(True)

    monoLeft = pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_B)
    monoRight = pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_C)
    stereo = pipeline.create(dai.node.StereoDepth)

    # Linking
    monoLeftOut = monoLeft.requestFullResolutionOutput()
    monoRightOut = monoRight.requestFullResolutionOutput()
    monoLeftOut.link(stereo.left)
    monoRightOut.link(stereo.right)

    stereo.setRectification(True)
    stereo.setExtendedDisparity(True)
    stereo.setLeftRightCheck(True)

    disparityQueue = stereo.disparity.createOutputQueue()

    colorMap = cv2.applyColorMap(np.arange(256, dtype=np.uint8), cv2.COLORMAP_JET)
    colorMap[0] = [0, 0, 0]  # to make zero-disparity pixels black

    pipeline.start()
    maxDisparity = 1
    while pipeline.isRunning():
        disparity = disparityQueue.get()
        assert isinstance(disparity, dai.ImgFrame)
        npDisparity = disparity.getFrame()
        maxDisparity = max(maxDisparity, np.max(npDisparity))
        colorizedDisparity = cv2.applyColorMap(((npDisparity / maxDisparity) * 255).astype(np.uint8), colorMap)
        cv2.imshow("disparity", colorizedDisparity)
        key = cv2.waitKey(1)
        if key == ord('q'):
            pipeline.stop()
            break
        elif key == ord('s'):
            try:
                # If pipeline debugging is disabled, this will raise an exception
                pipelineState = pipeline.getPipelineState().nodes().detailed()
                for nodeId, nodeState in pipelineState.nodeStates.items():
                    nodeName = pipeline.getNode(nodeId).getName()
                    print(f"\n# State for node {pipeline.getNode(nodeId).getName()} ({nodeId}):")
                    print(f"## State: {nodeState.state}")
                    print(f"## mainLoopTiming: {'invalid' if not nodeState.mainLoopTiming.isValid() else ''}")
                    if(nodeState.mainLoopTiming.isValid()):
                        print("-----")
                        print(nodeState.mainLoopTiming)
                        print("-----")
                    print(f"## inputsGetTiming: {'invalid' if not nodeState.inputsGetTiming.isValid() else ''}")
                    if(nodeState.inputsGetTiming.isValid()):
                        print("-----")
                        print(nodeState.inputsGetTiming)
                        print("-----")
                    print(f"## outputsSendTiming: {'invalid' if not nodeState.outputsSendTiming.isValid() else ''}")
                    if(nodeState.outputsSendTiming.isValid()):
                        print("-----")
                        print(nodeState.outputsSendTiming)
                        print("-----")
                    print(f"## inputStates: {'empty' if not nodeState.inputStates else ''}")
                    for inputName, inputState in nodeState.inputStates.items():
                        if inputState.isValid():
                            print(f"### {inputName}:\n-----{inputState}\n-----")
                        else:
                            print(f"### {inputName}: invalid")
                    print(f"## outputStates: {'empty' if not nodeState.outputStates else ''}")
                    for outputName, outputState in nodeState.outputStates.items():
                        print(f"### {outputName}:\n-----{outputState}\n-----")
                    print(f"## otherTimings: {'empty' if not nodeState.otherTimings else ''}")
                    for otherName, otherTiming in nodeState.otherTimings.items():
                        print(f"### {otherName}:\n-----{otherTiming}\n-----")
            except Exception as e:
                print("Error getting pipeline state:", e)
