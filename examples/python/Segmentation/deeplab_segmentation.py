from typing import Any, Dict

import depthai as dai
import numpy as np

class SegmentationParser(dai.node.ThreadedHostNode):

    def __init__(
        self, output_layer_name: str = "output"
    ) -> None:
       
        super().__init__()
        self._input = self.createInput()
        self._out = self.createOutput()
        
        self.output_layer_name = output_layer_name

    def run(self):
        while self.isRunning():
            try:
                output = self._input.get()
            except dai.MessageQueue.QueueException:
                break  # Pipeline was stopped
            
            assert isinstance(output, dai.NNData)
            layers = output.getAllLayerNames()
            self.output_layer_name = layers[0]
            
            segmentation_mask = output.getTensor(
                self.output_layer_name, dequantize=True
            )
            assert isinstance(segmentation_mask, np.ndarray)
            if len(segmentation_mask.shape) == 4:
                segmentation_mask = segmentation_mask[0]
                
            print(f"Segmentation mask shape: {segmentation_mask.shape}")
            mask_shape = segmentation_mask.shape
            min_dim = np.argmin(mask_shape)
            
            if min_dim == len(mask_shape) - 1:
                segmentation_mask = segmentation_mask.transpose(2, 0, 1) # (C, H, W)

            class_map = (
                np.argmax(segmentation_mask, axis=0)
                .reshape(segmentation_mask.shape[1], segmentation_mask.shape[2])
                .astype(np.uint8)
            )
            
            class_map_vector = class_map.flatten().tolist()
            
            # mask_message = dai.SegmentationMask()

            
            mask_message = dai.Buffer()
            transformation = output.getTransformation()

            self._out.send(mask_message)
