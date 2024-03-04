//  To parse this JSON data, first install
//
//      json.hpp  https://github.com/nlohmann/json
//
//  Then include this file, and then do
//
//     NnArchiveConfig.hpp data = nlohmann::json::parse(jsonString);

#pragma once

#include "tl/optional.hpp"
#include "nlohmann/json.hpp"
#include "helper.hpp"

#include "Model.hpp"

namespace dai {
namespace json_types {
    enum class ConfigVersion : int;
}
}

namespace dai {
namespace json_types {
    /**
     * The main class of the multi/single-stage model config scheme (multi- stage models
     * consists of interconnected single-stage models).
     *
     * @type config_version: str
     * @ivar config_version: Static variable representing the version of the config scheme.
     * @type stages: list
     * @ivar stages: List of Model objects each representing a stage in the model (list of
     * one element for single-stage models).
     * @type connections: list
     * @ivar connections: List of connections instructing how to connect multi stage models
     * (empty for single-stage models).
     */

    using nlohmann::json;

    /**
     * The main class of the multi/single-stage model config scheme (multi- stage models
     * consists of interconnected single-stage models).
     *
     * @type config_version: str
     * @ivar config_version: Static variable representing the version of the config scheme.
     * @type stages: list
     * @ivar stages: List of Model objects each representing a stage in the model (list of
     * one element for single-stage models).
     * @type connections: list
     * @ivar connections: List of connections instructing how to connect multi stage models
     * (empty for single-stage models).
     */
    struct NnArchiveConfig {
        /**
         * Static variable representing the version of the config scheme.
         */
        ConfigVersion configVersion;
        /**
         * List of connections instructing how to connect multi stage models (empty for single-stage
         * models).
         */
        tl::optional<std::vector<std::string>> connections;
        /**
         * List of Model objects each representing a stage in the model (list of one element for
         * single-stage models)
         */
        std::vector<Model> stages;
    };
}
}
