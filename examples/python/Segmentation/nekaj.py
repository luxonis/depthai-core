import time
import depthai as dai

import numpy as np
from depthai_nodes.node import ParsingNeuralNetwork
from depthai_nodes.message import ImgDetectionExtended, ImgDetectionsExtended, Keypoint
import cv2

print(dai.__file__)

if __name__ == "__main__":
    frame_type = dai.ImgFrame.Type.BGR888i
    fps =  30
    device = dai.Device()
    print("device type:", device.getPlatform())
    
    model_description = dai.NNModelDescription("luxonis/yolov8-instance-segmentation-nano:coco-512x288", "RVC4")
    archive = dai.NNArchive(dai.getModelFromZoo(model_description))
    
    with dai.Pipeline(device) as pipeline:
        print("Creating pipeline...")

        input_node = pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_A)
        
        det_nn: ParsingNeuralNetwork = pipeline.create(ParsingNeuralNetwork).build(
            input_node, archive, fps=fps
        )
        
        
        output_queue = det_nn.passthrough.createOutputQueue()
        output_dets = det_nn.out.createOutputQueue()        
        
        print("Pipeline created.")

        pipeline.start()

        while pipeline.isRunning():
            in_frame = output_queue.get()
            dets = output_dets.get()

            assert isinstance(in_frame, dai.ImgFrame)
            assert isinstance(dets, ImgDetectionsExtended)
            image = in_frame.getCvFrame()
            
            for det in dets.detections:
                outer_points = det.rotated_rect
                outer_points = outer_points.denormalize(image.shape[1], image.shape[0])
                outer_points = outer_points.getPoints()
                outer_points = [[int(p.x), int(p.y)] for p in outer_points]
                outer_points = np.array(outer_points, dtype=np.int32)
                cv2.polylines(image, [outer_points], isClosed=True, color=(0, 255, 0), thickness=2)
            
            
            cv2.imshow("depth", image)
            key = cv2.waitKey(1)
        
        
        