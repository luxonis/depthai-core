//  To parse this JSON data, first install
//
//      json.hpp  https://github.com/nlohmann/json
//
//  Then include this file, and then do
//
//     ConfigVersion.hpp data = nlohmann::json::parse(jsonString);

#pragma once

#include "tl/optional.hpp"
#include <nlohmann/json.hpp>
#include "helper.hpp"

namespace dai {
namespace json_types {
    using nlohmann::json;

    enum class ConfigVersion : int { THE_10 };
}
}
