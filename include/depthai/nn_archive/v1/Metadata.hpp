#pragma once

#include <nlohmann/json.hpp>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include "depthai/common/json.hpp"
#include "depthai/common/optional.hpp"

namespace dai {
namespace nn_archive {
namespace v1 {
/**
 * Metadata of the parser.
 *
 * Metadata for the object detection head.
 *
 * @type classes: list
 * @ivar classes: Names of object classes detected by the model.
 * @type n_classes: int
 * @ivar n_classes: Number of object classes detected by the model.
 * @type iou_threshold: float
 * @ivar iou_threshold: Non-max supression threshold limiting boxes
 * intersection.
 * @type conf_threshold: float
 * @ivar conf_threshold: Confidence score threshold above which a
 * detected object is considered valid.
 * @type max_det: int
 * @ivar max_det: Maximum detections per image.
 * @type anchors: list
 * @ivar anchors: Predefined bounding boxes of different sizes and
 * aspect ratios. The innermost lists are length 2 tuples of box
 * sizes. The middle lists are anchors for each output. The outmost
 * lists go from smallest to largest output.
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
 * @ivar boxes_outputs: Output name corresponding to predicted bounding
 * box coordinates.
 * @type scores_outputs: str
 * @ivar scores_outputs: Output name corresponding to predicted
 * bounding box confidence scores.
 *
 * Metadata for the segmentation head.
 *
 * @type classes: list
 * @ivar classes: Names of object classes segmented by the model.
 * @type n_classes: int
 * @ivar n_classes: Number of object classes segmented by the model.
 * @type is_softmax: bool
 * @ivar is_softmax: True, if output is already softmaxed
 * @type background_class: bool | None
 * @ivar background_class: True, if class index 0 is treated as background.
 *
 * Metadata for the YOLO head.
 *
 * @type yolo_outputs: list
 * @ivar yolo_outputs: A list of output names for each of the different
 * YOLO grid sizes.
 * @type mask_outputs: list | None
 * @ivar mask_outputs: A list of output names for each mask output.
 * @type protos_outputs: str | None
 * @ivar protos_outputs: Output name for the protos.
 * @type keypoints_outputs: list | None
 * @ivar keypoints_outputs: A list of output names for the keypoints.
 * @type angles_outputs: list | None
 * @ivar angles_outputs: A list of output names for the angles.
 * @type subtype: str
 * @ivar subtype: YOLO family decoding subtype (e.g. yolov5, yolov6,
 * yolov7 etc.)
 * @type n_prototypes: int | None
 * @ivar n_prototypes: Number of prototypes per bbox in YOLO instance
 * segmnetation.
 * @type n_keypoints: int | None
 * @ivar n_keypoints: Number of keypoints per bbox in YOLO keypoint
 * detection.
 * @type is_softmax: bool | None
 * @ivar is_softmax: True, if output is already softmaxed in YOLO
 * instance segmentation
 * @type strides: list | None
 * @ivar strides: Strides for each YOLO output.
 *
 * Metadata for the basic head. It allows you to specify additional
 * fields.
 *
 * @type postprocessor_path: str | None
 * @ivar postprocessor_path: Path to the postprocessor.
 */

/**
 * Metadata of the parser.
 *
 * Metadata for the object detection head.
 *
 * @type classes: list
 * @ivar classes: Names of object classes detected by the model.
 * @type n_classes: int
 * @ivar n_classes: Number of object classes detected by the model.
 * @type iou_threshold: float
 * @ivar iou_threshold: Non-max supression threshold limiting boxes
 * intersection.
 * @type conf_threshold: float
 * @ivar conf_threshold: Confidence score threshold above which a
 * detected object is considered valid.
 * @type max_det: int
 * @ivar max_det: Maximum detections per image.
 * @type anchors: list
 * @ivar anchors: Predefined bounding boxes of different sizes and
 * aspect ratios. The innermost lists are length 2 tuples of box
 * sizes. The middle lists are anchors for each output. The outmost
 * lists go from smallest to largest output.
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
 * @ivar boxes_outputs: Output name corresponding to predicted bounding
 * box coordinates.
 * @type scores_outputs: str
 * @ivar scores_outputs: Output name corresponding to predicted
 * bounding box confidence scores.
 *
 * Metadata for the segmentation head.
 *
 * @type classes: list
 * @ivar classes: Names of object classes segmented by the model.
 * @type n_classes: int
 * @ivar n_classes: Number of object classes segmented by the model.
 * @type is_softmax: bool
 * @ivar is_softmax: True, if output is already softmaxed
 * @type background_class: bool | None
 * @ivar background_class: True, if class index 0 is treated as background.
 *
 * Metadata for the YOLO head.
 *
 * @type yolo_outputs: list
 * @ivar yolo_outputs: A list of output names for each of the different
 * YOLO grid sizes.
 * @type mask_outputs: list | None
 * @ivar mask_outputs: A list of output names for each mask output.
 * @type protos_outputs: str | None
 * @ivar protos_outputs: Output name for the protos.
 * @type keypoints_outputs: list | None
 * @ivar keypoints_outputs: A list of output names for the keypoints.
 * @type angles_outputs: list | None
 * @ivar angles_outputs: A list of output names for the angles.
 * @type subtype: str
 * @ivar subtype: YOLO family decoding subtype (e.g. yolov5, yolov6,
 * yolov7 etc.)
 * @type n_prototypes: int | None
 * @ivar n_prototypes: Number of prototypes per bbox in YOLO instance
 * segmnetation.
 * @type n_keypoints: int | None
 * @ivar n_keypoints: Number of keypoints per bbox in YOLO keypoint
 * detection.
 * @type is_softmax: bool | None
 * @ivar is_softmax: True, if output is already softmaxed in YOLO
 * instance segmentation
 * @type strides: list | None
 * @ivar strides: Strides for each YOLO output.
 *
 * Metadata for the basic head. It allows you to specify additional
 * fields.
 *
 * @type postprocessor_path: str | None
 * @ivar postprocessor_path: Path to the postprocessor.
 */
struct Metadata {
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
     * Path to the postprocessor.
     */
    std::optional<std::string> postprocessorPath;
    /**
     * True, if output is already softmaxed.
     *
     * True, if output is already softmaxed in YOLO instance segmentation.
     */
    std::optional<bool> isSoftmax;
    /**
     * True, if class index 0 is treated as background.
     */
    std::optional<bool> backgroundClass;
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
    std::optional<std::string> subtype;
    /**
     * A list of output names for each of the different YOLO grid sizes.
     */
    std::optional<std::vector<std::string>> yoloOutputs;
    /**
     * Strides for each YOLO output.
     */
    std::optional<std::vector<int64_t>> strides;

    /**
     * Additional parameters
     */
    nlohmann::json extraParams;
};

/** @brief Read Metadata from its NNArchive JSON representation. */
inline void from_json(const nlohmann::json& j, Metadata& value) {
    Metadata metadata;
    if(j.contains("anchors")) j.at("anchors").get_to(metadata.anchors);
    if(j.contains("classes")) j.at("classes").get_to(metadata.classes);
    if(j.contains("conf_threshold")) j.at("conf_threshold").get_to(metadata.confThreshold);
    if(j.contains("iou_threshold")) j.at("iou_threshold").get_to(metadata.iouThreshold);
    if(j.contains("max_det")) j.at("max_det").get_to(metadata.maxDet);
    if(j.contains("n_classes")) j.at("n_classes").get_to(metadata.nClasses);
    if(j.contains("postprocessor_path")) j.at("postprocessor_path").get_to(metadata.postprocessorPath);
    if(j.contains("is_softmax")) j.at("is_softmax").get_to(metadata.isSoftmax);
    if(j.contains("background_class")) j.at("background_class").get_to(metadata.backgroundClass);
    if(j.contains("boxes_outputs")) j.at("boxes_outputs").get_to(metadata.boxesOutputs);
    if(j.contains("scores_outputs")) j.at("scores_outputs").get_to(metadata.scoresOutputs);
    if(j.contains("angles_outputs")) j.at("angles_outputs").get_to(metadata.anglesOutputs);
    if(j.contains("keypoints_outputs")) j.at("keypoints_outputs").get_to(metadata.keypointsOutputs);
    if(j.contains("mask_outputs")) j.at("mask_outputs").get_to(metadata.maskOutputs);
    if(j.contains("n_keypoints")) j.at("n_keypoints").get_to(metadata.nKeypoints);
    if(j.contains("n_prototypes")) j.at("n_prototypes").get_to(metadata.nPrototypes);
    if(j.contains("protos_outputs")) j.at("protos_outputs").get_to(metadata.protosOutputs);
    if(j.contains("subtype")) j.at("subtype").get_to(metadata.subtype);
    if(j.contains("yolo_outputs")) j.at("yolo_outputs").get_to(metadata.yoloOutputs);
    if(j.contains("strides")) j.at("strides").get_to(metadata.strides);
    metadata.extraParams = j;
    value = std::move(metadata);
}

/** @brief Write Metadata in NNArchive JSON format, preserving custom metadata fields. */
inline void to_json(nlohmann::json& j, const Metadata& value) {
    j = value.extraParams.is_object() ? value.extraParams : nlohmann::json::object();
    j["anchors"] = value.anchors;
    j["classes"] = value.classes;
    j["conf_threshold"] = value.confThreshold;
    j["iou_threshold"] = value.iouThreshold;
    j["max_det"] = value.maxDet;
    j["n_classes"] = value.nClasses;
    j["postprocessor_path"] = value.postprocessorPath;
    j["is_softmax"] = value.isSoftmax;
    j["background_class"] = value.backgroundClass;
    j["boxes_outputs"] = value.boxesOutputs;
    j["scores_outputs"] = value.scoresOutputs;
    j["angles_outputs"] = value.anglesOutputs;
    j["keypoints_outputs"] = value.keypointsOutputs;
    j["mask_outputs"] = value.maskOutputs;
    j["n_keypoints"] = value.nKeypoints;
    j["n_prototypes"] = value.nPrototypes;
    j["protos_outputs"] = value.protosOutputs;
    j["subtype"] = value.subtype;
    j["yolo_outputs"] = value.yoloOutputs;
    j["strides"] = value.strides;
}

NOP_EXTERNAL_STRUCTURE(Metadata,
                       anchors,
                       classes,
                       confThreshold,
                       iouThreshold,
                       maxDet,
                       nClasses,
                       postprocessorPath,
                       isSoftmax,
                       backgroundClass,
                       boxesOutputs,
                       scoresOutputs,
                       anglesOutputs,
                       keypointsOutputs,
                       maskOutputs,
                       nKeypoints,
                       nPrototypes,
                       protosOutputs,
                       subtype,
                       yoloOutputs,
                       strides,
                       extraParams);

}  // namespace v1
}  // namespace nn_archive
}  // namespace dai
