//  To parse this JSON data, first install
//
//      json.hpp  https://github.com/nlohmann/json
//
//  Then include this file, and then do
//
//     Input.hpp data = nlohmann::json::parse(jsonString);

#pragma once

#include "tl/optional.hpp"
#include "nlohmann/json.hpp"
#include "helper.hpp"

#include "PreprocessingBlock.hpp"

namespace dai {
namespace json_types {
    enum class DataType : int;
    enum class InputType : int;
}
}

namespace dai {
namespace json_types {
    /**
     * Represents input stream of a model.
     *
     * @type name: str
     * @ivar name: Name of the input layer.
     *
     * @type dtype: DataType
     * @ivar dtype: Data type of the input data (e.g., 'float32').
     *
     * @type input_type: InputType
     * @ivar input_type: Type of input data (e.g., 'image').
     *
     * @type shape: list
     * @ivar shape: Shape of the input data as a list of integers (e.g. [H,W], [H,W,C],
     * [BS,H,W,C], ...).
     *
     * @type preprocessing: PreprocessingBlock
     * @ivar preprocessing: Preprocessing steps applied to the input data.
     */

    using nlohmann::json;

    /**
     * Represents input stream of a model.
     *
     * @type name: str
     * @ivar name: Name of the input layer.
     *
     * @type dtype: DataType
     * @ivar dtype: Data type of the input data (e.g., 'float32').
     *
     * @type input_type: InputType
     * @ivar input_type: Type of input data (e.g., 'image').
     *
     * @type shape: list
     * @ivar shape: Shape of the input data as a list of integers (e.g. [H,W], [H,W,C],
     * [BS,H,W,C], ...).
     *
     * @type preprocessing: PreprocessingBlock
     * @ivar preprocessing: Preprocessing steps applied to the input data.
     */
    struct Input {
        /**
         * Data type of the input data (e.g., 'float32').
         */
        DataType dtype;
        /**
         * Type of input data (e.g., 'image').
         */
        InputType inputType;
        /**
         * Name of the input layer.
         */
        std::string name;
        /**
         * Preprocessing steps applied to the input data.
         */
        PreprocessingBlock preprocessing;
        /**
         * Shape of the input data as a list of integers (e.g. [H,W], [H,W,C], [BS,H,W,C], ...).
         */
        std::vector<int64_t> shape;
    };
}
}
