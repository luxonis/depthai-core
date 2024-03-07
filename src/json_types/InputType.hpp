//  To parse this JSON data, first install
//
//      json.hpp  https://github.com/nlohmann/json
//
//  Then include this file, and then do
//
//     InputType.hpp data = nlohmann::json::parse(jsonString);

#pragma once

#include <optional>
#include <nlohmann/json.hpp>
#include "helper.hpp"

namespace dai {
namespace json_types {
    /**
     * Type of input data (e.g., 'image').
     *
     * Represents a type of input the model is expecting.
     */

    using nlohmann::json;

    /**
     * Type of input data (e.g., 'image').
     *
     * Represents a type of input the model is expecting.
     */
    enum class InputType : int { IMAGE, RAW };
}
}
