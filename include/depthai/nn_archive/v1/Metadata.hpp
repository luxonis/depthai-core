#pragma once

#include <vector>
#include <string>
#include <optional>

namespace dai {
namespace nn_archive {
namespace v1 {
    enum class ObjectDetectionSubtypeYolo : int;
}
}
}

namespace dai {
namespace nn_archive {
namespace v1 {
    /**
     * Metadata of the parser.
     *
     * Metadata for the basic head. It allows you to specify additional fields.
     *
     * @type postprocessor_path: str | None
     * @ivar postprocessor_path: Path to the postprocessor.
     *
     * Metadata for the object detection head.
     *
     * @type classes: list
     * @ivar classes: Names of object classes detected by the model.
     * @type n_classes: int
     * @ivar n_classes: Number of object classes detected by the model.
     * @type iou_threshold: float
     * @ivar iou_threshold: Non-max supression threshold limiting boxes intersection.
     * @type conf_threshold: float
     * @ivar conf_threshold: Confidence score threshold above which a detected object is
     * considered valid.
     * @type max_det: int
     * @ivar max_det: Maximum detections per image.
     * @type anchors: list
     * @ivar anchors: Predefined bounding boxes of different sizes and aspect ratios. The
     * innermost lists are length 2 tuples of box sizes. The middle lists are anchors
     * for each output. The outmost lists go from smallest to largest output.
     *
     * Metadata for the classification head.
     *
     * @type classes: list
     * @ivar classes: Names of object classes classified by the model.
     * @type n_classes: int
     * @ivar n_classes: Number of object classes classified by the model.
     * @type is_softmax: bool
     * @ivar is_softmax: True, if output is already softmaxed
     *
     * Metadata for the SSD object detection head.
     *
     * @type boxes_outputs: str
     * @ivar boxes_outputs: Output name corresponding to predicted bounding box
     * coordinates.
     * @type scores_outputs: str
     * @ivar scores_outputs: Output name corresponding to predicted bounding box confidence
     * scores.
     *
     * Metadata for the segmentation head.
     *
     * @type classes: list
     * @ivar classes: Names of object classes segmented by the model.
     * @type n_classes: int
     * @ivar n_classes: Number of object classes segmented by the model.
     * @type is_softmax: bool
     * @ivar is_softmax: True, if output is already softmaxed
     *
     * Metadata for the YOLO head.
     *
     * @type yolo_outputs: list
     * @ivar yolo_outputs: A list of output names for each of the different YOLO grid
     * sizes.
     * @type mask_outputs: list | None
     * @ivar mask_outputs: A list of output names for each mask output.
     * @type protos_outputs: str | None
     * @ivar protos_outputs: Output name for the protos.
     * @type keypoints_outputs: list | None
     * @ivar keypoints_outputs: A list of output names for the keypoints.
     * @type angles_outputs: list | None
     * @ivar angles_outputs: A list of output names for the angles.
     * @type subtype: C{ObjectDetectionSubtypeYOLO}
     * @ivar subtype: YOLO family decoding subtype (e.g. yolov5, yolov6, yolov7 etc.)
     * @type n_prototypes: int | None
     * @ivar n_prototypes: Number of prototypes per bbox in YOLO instance segmnetation.
     * @type n_keypoints: int | None
     * @ivar n_keypoints: Number of keypoints per bbox in YOLO keypoint detection.
     * @type is_softmax: bool | None
     * @ivar is_softmax: True, if output is already softmaxed in YOLO instance segmentation
     */


    /**
     * Metadata of the parser.
     *
     * Metadata for the basic head. It allows you to specify additional fields.
     *
     * @type postprocessor_path: str | None
     * @ivar postprocessor_path: Path to the postprocessor.
     *
     * Metadata for the object detection head.
     *
     * @type classes: list
     * @ivar classes: Names of object classes detected by the model.
     * @type n_classes: int
     * @ivar n_classes: Number of object classes detected by the model.
     * @type iou_threshold: float
     * @ivar iou_threshold: Non-max supression threshold limiting boxes intersection.
     * @type conf_threshold: float
     * @ivar conf_threshold: Confidence score threshold above which a detected object is
     * considered valid.
     * @type max_det: int
     * @ivar max_det: Maximum detections per image.
     * @type anchors: list
     * @ivar anchors: Predefined bounding boxes of different sizes and aspect ratios. The
     * innermost lists are length 2 tuples of box sizes. The middle lists are anchors
     * for each output. The outmost lists go from smallest to largest output.
     *
     * Metadata for the classification head.
     *
     * @type classes: list
     * @ivar classes: Names of object classes classified by the model.
     * @type n_classes: int
     * @ivar n_classes: Number of object classes classified by the model.
     * @type is_softmax: bool
     * @ivar is_softmax: True, if output is already softmaxed
     *
     * Metadata for the SSD object detection head.
     *
     * @type boxes_outputs: str
     * @ivar boxes_outputs: Output name corresponding to predicted bounding box
     * coordinates.
     * @type scores_outputs: str
     * @ivar scores_outputs: Output name corresponding to predicted bounding box confidence
     * scores.
     *
     * Metadata for the segmentation head.
     *
     * @type classes: list
     * @ivar classes: Names of object classes segmented by the model.
     * @type n_classes: int
     * @ivar n_classes: Number of object classes segmented by the model.
     * @type is_softmax: bool
     * @ivar is_softmax: True, if output is already softmaxed
     *
     * Metadata for the YOLO head.
     *
     * @type yolo_outputs: list
     * @ivar yolo_outputs: A list of output names for each of the different YOLO grid
     * sizes.
     * @type mask_outputs: list | None
     * @ivar mask_outputs: A list of output names for each mask output.
     * @type protos_outputs: str | None
     * @ivar protos_outputs: Output name for the protos.
     * @type keypoints_outputs: list | None
     * @ivar keypoints_outputs: A list of output names for the keypoints.
     * @type angles_outputs: list | None
     * @ivar angles_outputs: A list of output names for the angles.
     * @type subtype: C{ObjectDetectionSubtypeYOLO}
     * @ivar subtype: YOLO family decoding subtype (e.g. yolov5, yolov6, yolov7 etc.)
     * @type n_prototypes: int | None
     * @ivar n_prototypes: Number of prototypes per bbox in YOLO instance segmnetation.
     * @type n_keypoints: int | None
     * @ivar n_keypoints: Number of keypoints per bbox in YOLO keypoint detection.
     * @type is_softmax: bool | None
     * @ivar is_softmax: True, if output is already softmaxed in YOLO instance segmentation
     */
    struct Metadata {
        /**
         * Path to the postprocessor.
         */
        std::optional<std::string> postprocessorPath;
        /**
         * Predefined bounding boxes of different sizes and aspect ratios. The innermost lists are
         * length 2 tuples of box sizes. The middle lists are anchors for each output. The outmost
         * lists go from smallest to largest output.
         */
        std::optional<std::vector<std::vector<std::vector<double>>>> anchors;
        /**
         * Names of object classes recognized by the model.
         */
        std::optional<std::vector<std::string>> classes;
        /**
         * Confidence score threshold above which a detected object is considered valid.
         */
        std::optional<double> confThreshold;
        /**
         * Non-max supression threshold limiting boxes intersection.
         */
        std::optional<double> iouThreshold;
        /**
         * Maximum detections per image.
         */
        std::optional<int64_t> maxDet;
        /**
         * Number of object classes recognized by the model.
         */
        std::optional<int64_t> nClasses;
        /**
         * True, if output is already softmaxed.
         *
         * True, if output is already softmaxed in YOLO instance segmentation.
         */
        std::optional<bool> isSoftmax;
        /**
         * Output name corresponding to predicted bounding box coordinates.
         */
        std::optional<std::string> boxesOutputs;
        /**
         * Output name corresponding to predicted bounding box confidence scores.
         */
        std::optional<std::string> scoresOutputs;
        /**
         * A list of output names for the angles.
         */
        std::optional<std::vector<std::string>> anglesOutputs;
        /**
         * A list of output names for the keypoints.
         */
        std::optional<std::vector<std::string>> keypointsOutputs;
        /**
         * A list of output names for each mask output.
         */
        std::optional<std::vector<std::string>> maskOutputs;
        /**
         * Number of keypoints per bbox in YOLO keypoint detection.
         */
        std::optional<int64_t> nKeypoints;
        /**
         * Number of prototypes per bbox in YOLO instance segmnetation.
         */
        std::optional<int64_t> nPrototypes;
        /**
         * Output name for the protos.
         */
        std::optional<std::string> protosOutputs;
        /**
         * YOLO family decoding subtype (e.g. yolov5, yolov6, yolov7 etc.).
         */
        std::optional<ObjectDetectionSubtypeYolo> subtype;
        /**
         * A list of output names for each of the different YOLO grid sizes.
         */
        std::optional<std::vector<std::string>> yoloOutputs;
    };
}
}
}
