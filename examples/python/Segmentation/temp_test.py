import depthai as dai 


from pathlib import Path
import time
import depthai as dai

import numpy as np
from depthai_nodes.node import ParsingNeuralNetwork
from util_nodes import *

# visualizer = dai.RemoteConnection(httpPort=8082)

print(dai.__file__)

if __name__ == "__main__":
    frame_type = dai.ImgFrame.Type.BGR888i
    fps =  30
    device = dai.Device()
    print("device type:", device.getPlatform())
    
    with dai.Pipeline(device) as pipeline:
        print("Creating pipeline...")

        input_node = pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_A)
        
        cam_out = input_node.requestOutput((1280, 720), dai.ImgFrame.Type.BGR888i, fps=fps)
        cam_small = input_node.requestOutput((100, 100), dai.ImgFrame.Type.BGR888i, fps=fps)
        
        portNode = pipeline.create(PortToDaiDetections2)
        cam_small.link(portNode.input)
        
        overlay_node = pipeline.create(dai.node.Overlay)
        cam_out.link(overlay_node.inputFrame)
        portNode.output.link(overlay_node.inputDetections)

        # encoder = pipeline.create(dai.node.VideoEncoder).build(overlay_node.out, frameRate=fps, profile=dai.VideoEncoderProperties.Profile.H264_MAIN)
        # encoder.setMaxOutputFrameSize(1920 * 1088 * 3)
        
        
        # visualizer.addTopic("masked_frame", encoder.out)
        # visualizer.addTopic("color_frame", cam_out)
        q = overlay_node.out.createOutputQueue()
        
        print("Pipeline created.")

        pipeline.start()
        # visualizer.registerPipeline(pipeline)

        while pipeline.isRunning():
            f = q.get()
            cv2.imshow("overlay", f.getCvFrame())
            if cv2.waitKey(1) == ord('q'):
                break
            # key = visualizer.waitKey(1)
            # if key == ord("q"):
            #     print("Got q key. Exiting...")
            #     break
        
        