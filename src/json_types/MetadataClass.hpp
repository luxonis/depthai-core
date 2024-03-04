//  To parse this JSON data, first install
//
//      json.hpp  https://github.com/nlohmann/json
//
//  Then include this file, and then do
//
//     MetadataClass.hpp data = nlohmann::json::parse(jsonString);

#pragma once

#include "tl/optional.hpp"
#include "nlohmann/json.hpp"
#include "helper.hpp"

namespace dai {
namespace json_types {
    /**
     * Metadata object defining the model metadata.
     *
     * Represents metadata of a model.
     *
     * @type name: str
     * @ivar name: Name of the model.
     * @type path: str
     * @ivar path: Relative path to the model executable.
     */

    using nlohmann::json;

    /**
     * Metadata object defining the model metadata.
     *
     * Represents metadata of a model.
     *
     * @type name: str
     * @ivar name: Name of the model.
     * @type path: str
     * @ivar path: Relative path to the model executable.
     */
    struct MetadataClass {
        /**
         * Name of the model.
         */
        std::string name;
        /**
         * Relative path to the model executable.
         */
        std::string path;
    };
}
}
