from argparse import ArgumentParser
import math
from typing import  List
import cv2
import depthai as dai
import numpy as np

class PalmDetectionParser(dai.node.ThreadedHostNode):
    """
    Official MediaPipe Hands solution:
    https://ai.google.dev/edge/mediapipe/solutions/vision/hand_landmarker
    """

    def __init__(
        self,
        output_layer_names: List[str] = None,
        conf_threshold: float = 0.5,
        iou_threshold: float = 0.5,
        max_det: int = 100,
        scale: int = 192,
    ) -> None:
        
        super().__init__()
        self.input = self.createInput()
        self.out = self.createOutput()
        
        self.conf_threshold = conf_threshold
        self.iou_threshold = iou_threshold
        self.max_det = max_det
        self.output_layer_names = (
            [] if output_layer_names is None else output_layer_names
        )
        self.conf_threshold = conf_threshold
        self.iou_threshold = iou_threshold
        self.max_det = max_det
        self.scale = scale
        self.label_names = ["Palm"]
        self._anchors = self._generate_anchors(scale, scale)

    def run(self):
        while self.isRunning():
            try:
                output = self.input.get()
                assert(isinstance(output, dai.NNData))
            except dai.MessageQueue.QueueException:
                break  # Pipeline was stopped

            all_tensors = output.getAllLayerNames()

            bboxes = None
            scores = None

            for tensor_name in all_tensors:
                tensor = np.array(
                    output.getTensor(tensor_name, dequantize=True), dtype=np.float32
                )

                if bboxes is None or scores is None:
                    bboxes = tensor
                    scores = tensor
                else:
                    bboxes = bboxes if tensor.shape[-1] < bboxes.shape[-1] else tensor
                    scores = tensor if tensor.shape[-1] < scores.shape[-1] else scores

            if bboxes is None or scores is None:
                raise ValueError("No valid output tensors found.")
            
            bboxes = bboxes.reshape(-1, 18)
            scores = scores.reshape(-1)


            filtered_bboxes, final_scores, keypoints, angles = self.decode(
                bboxes=bboxes,
                scores=scores,
                anchors=self._anchors,
                score_thresh=self.conf_threshold,
                scale=self.scale,
            )


            indices = cv2.dnn.NMSBoxes(
                filtered_bboxes,
                final_scores,
                self.conf_threshold,
                self.iou_threshold,
                top_k=self.max_det,
            )
            bboxes = np.array(filtered_bboxes)[indices]
            scores = np.array(final_scores)[indices]
            angles = np.array(angles)[indices]
            keypoints = np.array(keypoints)[indices]
            bboxes = bboxes.astype(float) / self.scale

            bboxes = np.clip(bboxes, 0, 1)
            angles = np.round(angles, 0)

            labels = np.array([0] * len(bboxes))

            edges = [[0, 1], [0, 2], [0, 3], [0, 4], [0, 6]]
            
            detections = []
            for i, bbox in enumerate(bboxes):
                
                rect = dai.RotatedRect(dai.Point2f(bbox[0], bbox[1]), dai.Size2f(bbox[2], bbox[3]), angles[i])
                
                keypoints_list = dai.KeypointsList()
                keypoints_list.setKeypoints(dai.VectorPoint2f(keypoints[i]))
                keypoints_list.setEdges(edges)
                
                img_detection = dai.ImgDetection(boundingBox = rect, keypoints= keypoints_list, labelName = "Palm", label = 0, confidence = scores[i])
                detections.append(img_detection)
            
            detections_msg = dai.ImgDetections()
            detections_msg.detections = detections
            detections_msg.setTimestamp(output.getTimestamp())
            detections_msg.setSequenceNum(output.getSequenceNum())
            detections_msg.setTimestampDevice(output.getTimestampDevice())
            transformation = output.getTransformation()
            detections_msg.setTransformation(transformation)

            self.out.send(detections_msg)
            
            
    def _generate_anchors(self, input_size_width, input_size_height):
        """
        option : SSDAnchorOptions
         https://github.com/google/mediapipe/blob/master/mediapipe/calculators/tflite/ssd_anchors_calculator.cc
        """
        input_size_width = 192
        input_size_height = 192
        min_scale=0.1484375
        max_scale=0.75
        anchor_offset_x=0.5
        anchor_offset_y=0.5
        strides=[8, 16, 16, 16]
        aspect_ratios_first=[1.0]
        reduce_boxes_in_lowest_layer=False
        interpolated_scale_aspect_ratio=1.0
        fixed_anchor_size=True
        anchors = []
        layer_id = 0
        n_strides = len(strides)
        
        def calculate_scale(min_scale, max_scale, stride_index, num_strides):
            if num_strides == 1:
                return (min_scale + max_scale) / 2
            else:
                return min_scale + (max_scale - min_scale) * stride_index / (num_strides - 1)
        
        while layer_id < n_strides:
            anchor_height = []
            anchor_width = []
            aspect_ratios = []
            scales = []
            # For same strides, we merge the anchors in the same order.
            last_same_stride_layer = layer_id
            while (
                last_same_stride_layer < n_strides
                and strides[last_same_stride_layer] == strides[layer_id]
            ):
                scale = calculate_scale(
                    min_scale, max_scale, last_same_stride_layer, n_strides
                )
                if last_same_stride_layer == 0 and reduce_boxes_in_lowest_layer:
                    # For first layer, it can be specified to use predefined anchors.
                    aspect_ratios += [1.0, 2.0, 0.5]
                    scales += [0.1, scale, scale]
                else:
                    aspect_ratios += aspect_ratios_first
                    scales += [scale] * len(aspect_ratios)
                    if interpolated_scale_aspect_ratio > 0:
                        if last_same_stride_layer == n_strides - 1:
                            scale_next = 1.0
                        else:
                            scale_next = calculate_scale(
                                min_scale,
                                max_scale,
                                last_same_stride_layer + 1,
                                n_strides,
                            )
                        scales.append(math.sqrt(scale * scale_next))
                        aspect_ratios.append(interpolated_scale_aspect_ratio)
                last_same_stride_layer += 1

            for i, r in enumerate(aspect_ratios):
                ratio_sqrts = math.sqrt(r)
                anchor_height.append(scales[i] / ratio_sqrts)
                anchor_width.append(scales[i] * ratio_sqrts)

            stride = strides[layer_id]
            feature_map_height = math.ceil(input_size_height / stride)
            feature_map_width = math.ceil(input_size_width / stride)

            for y in range(feature_map_height):
                for x in range(feature_map_width):
                    for anchor_id in range(len(anchor_height)):
                        x_center = (x + anchor_offset_x) / feature_map_width
                        y_center = (y + anchor_offset_y) / feature_map_height
                        # new_anchor = Anchor(x_center=x_center, y_center=y_center)
                        if fixed_anchor_size:
                            new_anchor = [x_center, y_center, 1.0, 1.0]
                            # new_anchor.w = 1.0
                            # new_anchor.h = 1.0
                        else:
                            new_anchor = [
                                x_center,
                                y_center,
                                anchor_width[anchor_id],
                                anchor_height[anchor_id],
                            ]
                            # new_anchor.w = anchor_width[anchor_id]
                            # new_anchor.h = anchor_height[anchor_id]
                        anchors.append(new_anchor)

            layer_id = last_same_stride_layer
            
        anchors = np.array(anchors)
        return np.array(anchors)

    def decode(self, bboxes, scores, anchors, score_thresh, scale=192):
        # https://github.com/google/mediapipe/blob/master/mediapipe/modules/palm_detection/palm_detection_cpu.pbtxt :

        detections = []
        target_angle = math.pi * 0.5  # 90 = pi/2
        scores = 1 / (1 + np.exp(-scores))
        
        detection_mask = scores > score_thresh
        det_scores = scores[detection_mask]
        if det_scores.size == 0:
            return [], [], [], []
        
        det_bboxes2 = bboxes[detection_mask]
        det_anchors = anchors[detection_mask]


        det_bboxes = det_bboxes2 * np.tile(det_anchors[:, 2:4], 9) / scale + np.tile(
            det_anchors[:, 0:2], 9
        )
        det_bboxes[:, 2:4] = det_bboxes[:, 2:4] - det_anchors[:, 0:2]
        det_bboxes[:, 0:2] = det_bboxes[:, 0:2] - det_bboxes[:, 3:4] * 0.5

        filtered_bboxes = []
        final_scores = []
        keypoints = []
        angles = []
        
        def rotated_rect_to_points(cx, cy, w, h, rotation):
            b = math.cos(rotation) * 0.5
            a = math.sin(rotation) * 0.5
            p0x = cx - a * h - b * w
            p0y = cy + b * h - a * w
            p1x = cx + a * h - b * w
            p1y = cy - b * h - a * w
            p2x = int(2 * cx - p0x)
            p2y = int(2 * cy - p0y)
            p3x = int(2 * cx - p1x)
            p3y = int(2 * cy - p1y)
            p0x, p0y, p1x, p1y = int(p0x), int(p0y), int(p1x), int(p1y)
            return np.array([[p0x, p0y], [p1x, p1y], [p2x, p2y], [p3x, p3y]])

        
        for i in range(det_bboxes.shape[0]):
            box = det_bboxes[i, 0:4] * scale
            if box[2] < 0 or box[3] < 0: # negative width or height
                continue
            score = float(det_scores[i])
            kps = []
            # 0 : wrist
            # 1 : index finger joint
            # 2 : middle finger joint
            # 3 : ring finger joint
            # 4 : little finger joint
            # 5 :
            # 6 : thumb joint
            for kp in range(7):
                coordinates = det_bboxes[i, 4 + kp * 2 : 6 + kp * 2]
                kps.append(dai.Point2f(coordinates[0], coordinates[1]))
                
            wrist = kps[0]
            middle = kps[2]
            ring = kps[3]
            
            x0, y0 = wrist.x, wrist.y  # wrist center
            x1, y1 = middle.x, middle.y  # middle finger
            x2, y2 = ring.x, ring.x # ring finger
            avg_x = (x1 + x2) / 2
            avg_y = (y1 + y2) / 2
            rotation = target_angle - math.atan2(-(avg_y - y0), avg_x - x0)
            angle = rotation - 2 * math.pi * math.floor((rotation + math.pi) / (2 * math.pi)) # normalize radians
            angle = np.round(np.degrees(angle), 0)
            
            outer_points = rotated_rect_to_points(
                box[0] + box[2] / 2,
                box[1] + box[3] / 2,
                box[2],
                box[3],
                rotation,
            )
            
            cx, cy = np.mean(outer_points, axis=0)
            w = np.linalg.norm(outer_points[0] - outer_points[3])
            h = np.linalg.norm(outer_points[0] - outer_points[1])
                        
            filtered_bboxes.append([cx, cy, w, h])
            final_scores.append(score)
            keypoints.append(kps)
            angles.append(angle)
        
        return filtered_bboxes, final_scores, keypoints, angles

class CustomAnnotationNode(dai.node.ThreadedHostNode):
    def __init__(self):
        super().__init__()
        self.input = self.createInput()
        self.out = self.createOutput()
        
    def run(self):
        while self.isRunning():
            try:
                detection_msg = self.input.get()
                assert(isinstance(detection_msg, dai.ImgDetections))
            except dai.MessageQueue.QueueException:
                break  # Pipeline was stopped
            
            imgAnnotations = dai.ImgAnnotations()
            imgAnnotations.setTimestamp(detection_msg.getTimestamp())
            
            outline_color  = dai.Color(21 / 255, 127 / 255, 88/ 255, 1.0)
            fill_color  = dai.Color(21 / 255, 127 / 255, 88/ 255, 0.1)
            
            imgAnnotation = dai.ImgAnnotation()

            detections = detection_msg.detections
            for det in detections:
                # box
                bbox_annotation = dai.PointsAnnotation()
                bbox: dai.RotatedRect = det.getBoundingBox()
                corners = dai.VectorPoint2f(bbox.getPoints())
                
                bbox_annotation.points = corners
                bbox_annotation.outlineColor = outline_color
                bbox_annotation.fillColor = fill_color
                bbox_annotation.thickness = 1
                bbox_annotation.type = dai.PointsAnnotationType.LINE_LOOP
                imgAnnotation.points.append(bbox_annotation)
                
                # text
                
                text_annotation = dai.TextAnnotation()
                top_left = corners[1]
                
                text_annotation.position = top_left
                text_annotation.text = f"{det.labelName} {det.confidence*100:.2f}%"
                text_annotation.fontSize = 15
                text_annotation.textColor = outline_color
                imgAnnotation.texts.append(text_annotation)
                
                # keypoints
                
                keypoints: list[dai.Keypoint]= det.getKeypoints()
                edges: list[list] = det.getEdges()
                
                for edge_pair in edges:
                    keypoint1: dai.Point3f = keypoints[edge_pair[0]].coordinates
                    keypoint2: dai.Point3f = keypoints[edge_pair[1]].coordinates
                    
                    keypoint_1_coordinates = dai.Point2f(keypoint1.x, keypoint1.y)
                    keypoint_2_coordinates = dai.Point2f(keypoint2.x, keypoint2.y)
                    
                    edge_annotation = dai.PointsAnnotation()
                    edge_annotation.points = dai.VectorPoint2f([keypoint_1_coordinates, keypoint_2_coordinates])
                    edge_annotation.outlineColor = dai.Color(float(240 / 255), float(240 / 255), float(240 / 255), float(1.0))
                    edge_annotation.thickness = 1
                    edge_annotation.type = dai.PointsAnnotationType.LINE_STRIP
                    
                    imgAnnotation.points.append(edge_annotation)
                
                imgAnnotations.annotations.append(imgAnnotation)
                
            self.out.send(imgAnnotations)

parser = ArgumentParser()
parser.add_argument("--webSocketPort", type=int, default=8765)
parser.add_argument("--httpPort", type=int, default=8082)
args = parser.parse_args()

remoteConnector = dai.RemoteConnection(webSocketPort=args.webSocketPort, httpPort=args.httpPort)

with dai.Pipeline() as pipeline:
    cameraNode = pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_A)
    detectionNetwork = pipeline.create(dai.node.NeuralNetwork).build(
        cameraNode, dai.NNModelDescription("luxonis/mediapipe-palm-detection:192x192")
    )
    
    parser = pipeline.create(PalmDetectionParser)
    detectionNetwork.out.link(parser.input)
    
    customAnnotation = pipeline.create(CustomAnnotationNode)
    parser.out.link(customAnnotation.input)
    

    remoteConnector.addTopic("images", detectionNetwork.passthrough, "img")
    remoteConnector.addTopic("detections", parser.out, "img") # default visualizations still work due PR extending ImgDetections
    remoteConnector.addTopic("modified_annotations", customAnnotation.out, "img") # an example of new annotation style with rotated boxes and keypoints
    
    pipeline.start()
    remoteConnector.registerPipeline(pipeline)

    while pipeline.isRunning():
        key = remoteConnector.waitKey(1)
        if key == ord("q"):
            print("Got q key from the remote connection!")
            break
