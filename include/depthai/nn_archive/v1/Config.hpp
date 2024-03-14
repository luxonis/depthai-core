#pragma once

#include <optional>

#include "Model.hpp"

namespace dai {
namespace nn_archive {
namespace v1 {
    enum class ConfigVersion : int;
}
}
}

namespace dai {
namespace nn_archive {
namespace v1 {
    /**
     * The main class of the multi/single-stage model config scheme (multi- stage models
     * consists of interconnected single-stage models).
     *
     * @type config_version: str
     * @ivar config_version: Static variable representing the version of the config scheme.
     * @type model: Model
     * @ivar model: A Model object representing the neural network used in the archive.
     */


    /**
     * The main class of the multi/single-stage model config scheme (multi- stage models
     * consists of interconnected single-stage models).
     *
     * @type config_version: str
     * @ivar config_version: Static variable representing the version of the config scheme.
     * @type model: Model
     * @ivar model: A Model object representing the neural network used in the archive.
     */
    struct Config {
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
}
