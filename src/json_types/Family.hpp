//  To parse this JSON data, first install
//
//      json.hpp  https://github.com/nlohmann/json
//
//  Then include this file, and then do
//
//     Family.hpp data = nlohmann::json::parse(jsonString);

#pragma once

#include <optional>
#include <nlohmann/json.hpp>
#include "helper.hpp"

namespace dai {
namespace json_types {
    using nlohmann::json;

    enum class Family : int { CLASSIFICATION, INSTANCE_SEGMENTATION_YOLO, OBJECT_DETECTION_SSD, OBJECT_DETECTION_YOLO, SEGMENTATION };
}
}
