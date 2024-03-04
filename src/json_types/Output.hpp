//  To parse this JSON data, first install
//
//      json.hpp  https://github.com/nlohmann/json
//
//  Then include this file, and then do
//
//     Output.hpp data = nlohmann::json::parse(jsonString);

#pragma once

#include "tl/optional.hpp"
#include "nlohmann/json.hpp"
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
     * @type head_ids: list
     * @ivar head_ids: IDs of heads which accept this output stream (beware that a single
     * output can go into multiple heads).
     */

    using nlohmann::json;

    /**
     * Represents output stream of a model.
     *
     * @type name: str
     * @ivar name: Name of the output layer.
     * @type dtype: DataType
     * @ivar dtype: Data type of the output data (e.g., 'float32').
     * @type head_ids: list
     * @ivar head_ids: IDs of heads which accept this output stream (beware that a single
     * output can go into multiple heads).
     */
    struct Output {
        /**
         * Data type of the output data (e.g., 'float32').
         */
        DataType dtype;
        /**
         * IDs of heads which accept this output stream (beware that a single output can go into
         * multiple heads).
         */
        std::vector<std::string> headIds;
        /**
         * Name of the output layer.
         */
        std::string name;
    };
}
}
