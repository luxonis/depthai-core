//  To parse this JSON data, first install
//
//      json.hpp  https://github.com/nlohmann/json
//
//  Then include this file, and then do
//
//     NnArchiveConfig.hpp data = nlohmann::json::parse(jsonString);

#pragma once

#include <optional>
#include <nlohmann/json.hpp>
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
     * @type model: Model
     * @ivar model: A Model object representing the neural network used in the archive.
     */

    using nlohmann::json;

    /**
     * The main class of the multi/single-stage model config scheme (multi- stage models
     * consists of interconnected single-stage models).
     *
     * @type config_version: str
     * @ivar config_version: Static variable representing the version of the config scheme.
     * @type model: Model
     * @ivar model: A Model object representing the neural network used in the archive.
     */
    struct NnArchiveConfig {
        /**
         * Static variable representing the version of the config scheme.
         */
        ConfigVersion configVersion;
        /**
         * A Model object representing the neural network used in the archive.
         */
        Model model;
    };
}
}
