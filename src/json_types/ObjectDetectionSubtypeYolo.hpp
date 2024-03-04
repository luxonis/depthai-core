//  To parse this JSON data, first install
//
//      json.hpp  https://github.com/nlohmann/json
//
//  Then include this file, and then do
//
//     ObjectDetectionSubtypeYolo.hpp data = nlohmann::json::parse(jsonString);

#pragma once

#include "tl/optional.hpp"
#include "nlohmann/json.hpp"
#include "helper.hpp"

namespace dai {
namespace json_types {
    /**
     * YOLO family decoding subtype (e.g. v5, v6, v7 etc.).
     *
     * Object detection decoding subtypes for YOLO networks.
     *
     * Subtype members have exactly the same decoding.
     */

    using nlohmann::json;

    /**
     * YOLO family decoding subtype (e.g. v5, v6, v7 etc.).
     *
     * Object detection decoding subtypes for YOLO networks.
     *
     * Subtype members have exactly the same decoding.
     */
    enum class ObjectDetectionSubtypeYolo : int { YOLOV5, YOLOV6, YOLOV7, YOLOV8 };
}
}
