//  To parse this JSON data, first install
//
//      json.hpp  https://github.com/nlohmann/json
//
//  Then include this file, and then do
//
//     Output.hpp data = nlohmann::json::parse(jsonString);

#pragma once

#include <optional>
#include <nlohmann/json.hpp>
#include "helper.hpp"

namespace dai {
namespace json_types {
    enum class DataType : int;
}
}

namespace dai {
namespace json_types {
    /**
     * Represents output stream of a model.
     *
     * @type name: str
     * @ivar name: Name of the output layer.
     * @type dtype: DataType
     * @ivar dtype: Data type of the output data (e.g., 'float32').
     */

    using nlohmann::json;

    /**
     * Represents output stream of a model.
     *
     * @type name: str
     * @ivar name: Name of the output layer.
     * @type dtype: DataType
     * @ivar dtype: Data type of the output data (e.g., 'float32').
     */
    struct Output {
        /**
         * Data type of the output data (e.g., 'float32').
         */
        DataType dtype;
        /**
         * Name of the output layer.
         */
        std::string name;
    };
}
}
