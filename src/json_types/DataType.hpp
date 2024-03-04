//  To parse this JSON data, first install
//
//      json.hpp  https://github.com/nlohmann/json
//
//  Then include this file, and then do
//
//     DataType.hpp data = nlohmann::json::parse(jsonString);

#pragma once

#include "tl/optional.hpp"
#include <nlohmann/json.hpp>
#include "helper.hpp"

namespace dai {
namespace json_types {
    /**
     * Data type of the input data (e.g., 'float32').
     *
     * Represents all existing data types used in i/o streams of the model.
     *
     * Data type of the output data (e.g., 'float32').
     */

    using nlohmann::json;

    /**
     * Data type of the input data (e.g., 'float32').
     *
     * Represents all existing data types used in i/o streams of the model.
     *
     * Data type of the output data (e.g., 'float32').
     */
    enum class DataType : int { FLOAT16, FLOAT32, INT8, NV12, UINT8 };
}
}
