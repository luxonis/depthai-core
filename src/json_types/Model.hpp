//  To parse this JSON data, first install
//
//      json.hpp  https://github.com/nlohmann/json
//
//  Then include this file, and then do
//
//     Model.hpp data = nlohmann::json::parse(jsonString);

#pragma once

#include "tl/optional.hpp"
#include <nlohmann/json.hpp>
#include "helper.hpp"

#include "Head.hpp"
#include "Input.hpp"
#include "MetadataClass.hpp"
#include "Output.hpp"

namespace dai {
namespace json_types {
    /**
     * Class defining a single-stage model config scheme.
     *
     * @type metadata: Metadata
     * @ivar metadata: Metadata object defining the model metadata.
     * @type inputs: list
     * @ivar inputs: List of Input objects defining the model inputs.
     * @type outputs: list
     * @ivar outputs: List of Output objects defining the model outputs.
     * @type heads: list
     * @ivar heads: List of Head objects defining the model heads. If not defined, we
     * assume a raw output.
     */

    using nlohmann::json;

    /**
     * Class defining a single-stage model config scheme.
     *
     * @type metadata: Metadata
     * @ivar metadata: Metadata object defining the model metadata.
     * @type inputs: list
     * @ivar inputs: List of Input objects defining the model inputs.
     * @type outputs: list
     * @ivar outputs: List of Output objects defining the model outputs.
     * @type heads: list
     * @ivar heads: List of Head objects defining the model heads. If not defined, we
     * assume a raw output.
     */
    struct Model {
        /**
         * List of Head objects defining the model heads. If not defined, we assume a raw output.
         */
        tl::optional<std::vector<Head>> heads;
        /**
         * List of Input objects defining the model inputs.
         */
        std::vector<Input> inputs;
        /**
         * Metadata object defining the model metadata.
         */
        MetadataClass metadata;
        /**
         * List of Output objects defining the model outputs.
         */
        std::vector<Output> outputs;
    };
}
}
