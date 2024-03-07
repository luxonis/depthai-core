//  To parse this JSON data, first install
//
//      json.hpp  https://github.com/nlohmann/json
//
//  Then include this file, and then do
//
//     Head.hpp data = nlohmann::json::parse(jsonString);

#pragma once

#include <optional>
#include <nlohmann/json.hpp>
#include "helper.hpp"

#include "Metadata.hpp"

namespace dai {
namespace json_types {
    /**
     * Represents head of a model.
     *
     * @type outputs: C{Union[List[str], Dict[str, Union[str, List]]]}
     * @ivar outputs: A list of output names from the `outputs` block of the archive or a
     * dictionary mapping DepthAI parser names needed for the head to output names. The
     * referenced outputs will be used by the DepthAI parser.
     * @type metadata: HeadMetadata
     * @ivar metadata: Parameters required by head to run postprocessing.
     */

    using nlohmann::json;

    /**
     * Represents head of a model.
     *
     * @type outputs: C{Union[List[str], Dict[str, Union[str, List]]]}
     * @ivar outputs: A list of output names from the `outputs` block of the archive or a
     * dictionary mapping DepthAI parser names needed for the head to output names. The
     * referenced outputs will be used by the DepthAI parser.
     * @type metadata: HeadMetadata
     * @ivar metadata: Parameters required by head to run postprocessing.
     */
    struct Head {
        /**
         * Parameters required by head to run postprocessing.
         */
        Metadata metadata;
    };
}
}
