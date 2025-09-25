# import cv2
# import numpy as np
try: 
    while True:
        frame = node.inputs["frame"].get()
        # detections = node.inputs['detections'].get()
        
        # image = frame.getCvFrame()
        # mask = detections.getSegmentationMask().astype(np.int32)
        # mask = cv2.resize(mask, (1920, 1080), interpolation=cv2.INTER_NEAREST)
    
        # mask[mask == 255] = -1
        # scaled_mask = np.ones_like(mask, dtype=np.uint8) * 255
        # max_val = np.max(mask)
        # min_val = np.min(mask)
        
        # if min_val != max_val:
        #     scaled_mask = ((mask - min_val) / (max_val - min_val) * 255).astype(np.uint8)
        
        # scaled_mask[mask == 255] = 0
        # scaled_mask = cv2.applyColorMap(scaled_mask, cv2.COLORMAP_JET)
        # scaled_mask[mask == -1] = image[mask == -1]
        
        # alpha = 0.7
        # image = cv2.addWeighted( image, alpha, scaled_mask, 1 - alpha, 0)
        
        # masked_frame = ImgFrame()
        # masked_frame.setFrame(image)
        # masked_frame.setType(ImgFrame.Type.NV12)
        # masked_frame.setWidth(image.shape[1])
        # masked_frame.setHeight(image.shape[0])
        # masked_frame.setTimestamp(frame.getTimestamp())
        # masked_frame.setSequenceNum(frame.getSequenceNum())
        # masked_frame.setTimestampDevice(frame.getTimestampDevice())
        # masked_frame.setTransformation(frame.getTransformation())
        # node.warn(f'Processed frame {masked_frame.getSequenceNum()}')
        node.outputs["masked_frame"].send(frame)
        # node.outputs['masked_frame'].send(masked_frame)
except Exception as e:
        node.warn(f"Script node error: {e}")