//  To parse this JSON data, first install
//
//      json.hpp  https://github.com/nlohmann/json
//
//  Then include this file, and then do
//
//     Metadata.hpp data = nlohmann::json::parse(jsonString);

#pragma once

#include "tl/optional.hpp"
#include <nlohmann/json.hpp>
#include "helper.hpp"

namespace dai {
namespace json_types {
    enum class Family : int;
    enum class ObjectDetectionSubtypeYolo : int;
}
}

namespace dai {
namespace json_types {
    /**
     * Parameters required by head to run postprocessing.
     *
     * Metadata for YOLO object detection head.
     *
     * @type family: str
     * @ivar family: Decoding family.
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
     * @type anchors: list
     * @ivar anchors: Predefined bounding boxes of different sizes and aspect ratios.
     *
     * Metadata for segmentation head.
     *
     * @type family: str
     * @ivar family: Decoding family.
     * @type is_softmax: bool
     * @ivar is_softmax: True, if output is already softmaxed.
     *
     * Metadata for classification head.
     *
     * @type family: str
     * @ivar family: Decoding family.
     * @type is_softmax: bool
     * @ivar is_softmax: True, if output is already softmaxed.
     */

    using nlohmann::json;

    /**
     * Parameters required by head to run postprocessing.
     *
     * Metadata for YOLO object detection head.
     *
     * @type family: str
     * @ivar family: Decoding family.
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
     * @type anchors: list
     * @ivar anchors: Predefined bounding boxes of different sizes and aspect ratios.
     *
     * Metadata for segmentation head.
     *
     * @type family: str
     * @ivar family: Decoding family.
     * @type is_softmax: bool
     * @ivar is_softmax: True, if output is already softmaxed.
     *
     * Metadata for classification head.
     *
     * @type family: str
     * @ivar family: Decoding family.
     * @type is_softmax: bool
     * @ivar is_softmax: True, if output is already softmaxed.
     */
    struct Metadata {
        /**
         * Names of object classes recognized by the model.
         */
        std::vector<std::string> classes;
        /**
         * Confidence score threshold above which a detected object is considered valid.
         */
        tl::optional<double> confThreshold;
        /**
         * Decoding family.
         */
        Family family;
        /**
         * Non-max supression threshold limiting boxes intersection.
         */
        tl::optional<double> iouThreshold;
        /**
         * Maximum detections per image.
         */
        tl::optional<int64_t> maxDet;
        /**
         * Number of object classes recognized by the model.
         */
        int64_t nClasses;
        /**
         * Number of keypoints per bbox if provided.
         */
        tl::optional<int64_t> nKeypoints;
        /**
         * Number of prototypes per bbox if provided.
         */
        tl::optional<int64_t> nPrototypes;
        /**
         * Output node containing prototype information.
         */
        tl::optional<std::string> prototypeOutputName;
        /**
         * Step size at which the filter (or kernel) moves across the input data during convolution.
         */
        tl::optional<int64_t> stride;
        /**
         * YOLO family decoding subtype (e.g. v5, v6, v7 etc.).
         */
        tl::optional<ObjectDetectionSubtypeYolo> subtype;
        /**
         * Predefined bounding boxes of different sizes and aspect ratios.
         */
        tl::optional<std::vector<std::vector<int64_t>>> anchors;
        /**
         * True, if output is already softmaxed.
         */
        tl::optional<bool> isSoftmax;
    };
}
}
