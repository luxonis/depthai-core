#pragma once

#include <optional>

#include "Outputs.hpp"

namespace dai {
namespace nn_archive_v1 {
    enum class ObjectDetectionSubtypeYolo : int;
}
}

namespace dai {
namespace nn_archive_v1 {
    /**
     * Metadata for classification head.
     *
     * @type family: str
     * @ivar family: Decoding family.
     * @type outputs: C{OutputsClassification}
     * @ivar outputs: A configuration specifying which output names from the `outputs` block of
     * the archive are fed into the head.
     * @type is_softmax: bool
     * @ivar is_softmax: True, if output is already softmaxed.
     *
     * Metadata for object detection head.
     *
     * @type iou_threshold: float
     * @ivar iou_threshold: Non-max supression threshold limiting boxes intersection.
     * @type conf_threshold: float
     * @ivar conf_threshold: Confidence score threshold above which a detected object is
     * considered valid.
     * @type max_det: int
     * @ivar max_det: Maximum detections per image.
     * @type anchors: C{Optional[List[List[List[int]]]]}
     * @ivar anchors: Predefined bounding boxes of different sizes and aspect ratios. The
     * innermost lists are length 2 tuples of box sizes. The middle lists are anchors
     * for each output. The outmost lists go from smallest to largest output.
     *
     * Metadata for YOLO object detection head.
     *
     * @type family: str
     * @ivar family: Decoding family.
     * @type outputs: C{ObjectDetectionYOLO}
     * @ivar outputs: A configuration specifying which output names from the `outputs` block of
     * the archive are fed into the head.
     * @type subtype: ObjectDetectionSubtypeYOLO
     * @ivar subtype: YOLO family decoding subtype (e.g. v5, v6, v7 etc.).
     * @type n_keypoints: int
     * @ivar n_keypoints: Number of keypoints per bbox if provided.
     * @type n_prototypes: int
     * @ivar n_prototypes: Number of prototypes per bbox if provided.
     * @type prototype_output_name: str
     * @ivar prototype_output_name: Output node containing prototype information.
     *
     * Metadata for SSD object detection head.
     *
     * @type family: str
     * @ivar family: Decoding family.
     * @type outputs: C{OutputsSSD}
     * @ivar outputs: A configuration specifying which output names from the `outputs` block of
     * the archive are fed into the head.
     *
     * Metadata for segmentation head.
     *
     * @type family: str
     * @ivar family: Decoding family.
     * @type outputs: C{OutputsSegmentation}
     * @ivar outputs: A configuration specifying which output names from the `outputs` block of
     * the archive are fed into the head.
     * @type is_softmax: bool
     * @ivar is_softmax: True, if output is already softmaxed.
     *
     * Metadata for YOLO instance segmentation head.
     *
     * @type family: str
     * @ivar family: Decoding family.
     * @type outputs: C{OutputsInstanceSegmentationYOLO}
     * @ivar outputs: A configuration specifying which output names from the `outputs` block of
     * the archive are fed into the head.
     * @type postprocessor_path: str
     * @ivar postprocessor_path: Path to the secondary executable used in YOLO instance
     * segmentation.
     *
     * Metadata for YOLO keypoint detection head.
     *
     * @type family: str
     * @ivar family: Decoding family.
     * @type outputs: C{OutputsKeypointDetectionYOLO}
     * @ivar outputs: A configuration specifying which output names from the `outputs` block of
     * the archive are fed into the head.
     */


    /**
     * Metadata for classification head.
     *
     * @type family: str
     * @ivar family: Decoding family.
     * @type outputs: C{OutputsClassification}
     * @ivar outputs: A configuration specifying which output names from the `outputs` block of
     * the archive are fed into the head.
     * @type is_softmax: bool
     * @ivar is_softmax: True, if output is already softmaxed.
     *
     * Metadata for object detection head.
     *
     * @type iou_threshold: float
     * @ivar iou_threshold: Non-max supression threshold limiting boxes intersection.
     * @type conf_threshold: float
     * @ivar conf_threshold: Confidence score threshold above which a detected object is
     * considered valid.
     * @type max_det: int
     * @ivar max_det: Maximum detections per image.
     * @type anchors: C{Optional[List[List[List[int]]]]}
     * @ivar anchors: Predefined bounding boxes of different sizes and aspect ratios. The
     * innermost lists are length 2 tuples of box sizes. The middle lists are anchors
     * for each output. The outmost lists go from smallest to largest output.
     *
     * Metadata for YOLO object detection head.
     *
     * @type family: str
     * @ivar family: Decoding family.
     * @type outputs: C{ObjectDetectionYOLO}
     * @ivar outputs: A configuration specifying which output names from the `outputs` block of
     * the archive are fed into the head.
     * @type subtype: ObjectDetectionSubtypeYOLO
     * @ivar subtype: YOLO family decoding subtype (e.g. v5, v6, v7 etc.).
     * @type n_keypoints: int
     * @ivar n_keypoints: Number of keypoints per bbox if provided.
     * @type n_prototypes: int
     * @ivar n_prototypes: Number of prototypes per bbox if provided.
     * @type prototype_output_name: str
     * @ivar prototype_output_name: Output node containing prototype information.
     *
     * Metadata for SSD object detection head.
     *
     * @type family: str
     * @ivar family: Decoding family.
     * @type outputs: C{OutputsSSD}
     * @ivar outputs: A configuration specifying which output names from the `outputs` block of
     * the archive are fed into the head.
     *
     * Metadata for segmentation head.
     *
     * @type family: str
     * @ivar family: Decoding family.
     * @type outputs: C{OutputsSegmentation}
     * @ivar outputs: A configuration specifying which output names from the `outputs` block of
     * the archive are fed into the head.
     * @type is_softmax: bool
     * @ivar is_softmax: True, if output is already softmaxed.
     *
     * Metadata for YOLO instance segmentation head.
     *
     * @type family: str
     * @ivar family: Decoding family.
     * @type outputs: C{OutputsInstanceSegmentationYOLO}
     * @ivar outputs: A configuration specifying which output names from the `outputs` block of
     * the archive are fed into the head.
     * @type postprocessor_path: str
     * @ivar postprocessor_path: Path to the secondary executable used in YOLO instance
     * segmentation.
     *
     * Metadata for YOLO keypoint detection head.
     *
     * @type family: str
     * @ivar family: Decoding family.
     * @type outputs: C{OutputsKeypointDetectionYOLO}
     * @ivar outputs: A configuration specifying which output names from the `outputs` block of
     * the archive are fed into the head.
     */
    struct Head {
        /**
         * Names of object classes recognized by the model.
         */
        std::vector<std::string> classes;
        /**
         * Decoding family.
         */
        std::string family;
        /**
         * True, if output is already softmaxed.
         */
        std::optional<bool> isSoftmax;
        /**
         * Number of object classes recognized by the model.
         */
        int64_t nClasses;
        /**
         * A configuration specifying which output names from the `outputs` block of the archive are
         * fed into the head.
         */
        Outputs outputs;
        /**
         * Predefined bounding boxes of different sizes and aspect ratios. The innermost lists are
         * length 2 tuples of box sizes. The middle lists are anchors for each output. The outmost
         * lists go from smallest to largest output.
         */
        std::optional<std::vector<std::vector<std::vector<int64_t>>>> anchors;
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
         * Number of keypoints per bbox if provided.
         */
        std::optional<int64_t> nKeypoints;
        /**
         * Number of prototypes per bbox if provided.
         */
        std::optional<int64_t> nPrototypes;
        /**
         * Output node containing prototype information.
         */
        std::optional<std::string> prototypeOutputName;
        /**
         * YOLO family decoding subtype (e.g. v5, v6, v7 etc.).
         */
        std::optional<ObjectDetectionSubtypeYolo> subtype;
        /**
         * Path to the secondary executable used in YOLO instance segmentation.
         */
        std::optional<std::string> postprocessorPath;
    };
}
}
