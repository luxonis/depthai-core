//  To parse this JSON data, first install
//
//      json.hpp  https://github.com/nlohmann/json
//
//  Then include this file, and then do
//
//     Head.hpp data = nlohmann::json::parse(jsonString);

#pragma once

#include "tl/optional.hpp"
#include <nlohmann/json.hpp>
#include "helper.hpp"

#include "Metadata.hpp"

namespace dai {
namespace json_types {
    /**
     * Represents head of a model.
     *
     * @type head_id: str
     * @ivar head_id: Unique head identifier.
     * @type metadata: HeadMetadata
     * @ivar metadata: Parameters required by head to run postprocessing.
     */

    using nlohmann::json;

    /**
     * Represents head of a model.
     *
     * @type head_id: str
     * @ivar head_id: Unique head identifier.
     * @type metadata: HeadMetadata
     * @ivar metadata: Parameters required by head to run postprocessing.
     */
    struct Head {
        /**
         * Unique head identifier.
         */
        std::string headId;
        /**
         * Parameters required by head to run postprocessing.
         */
        Metadata metadata;
    };
}
}
