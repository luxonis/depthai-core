#pragma once

#include <optional>
#include <string>

#include "Model.hpp"

namespace dai {
namespace nn_archive {
namespace v1 {
/**
 * The main class of the multi/single-stage model config scheme
 * (multi- stage models consists of interconnected single-stage
 * models).
 *
 * @type config_version: str
 * @ivar config_version: String representing config schema version in
 * format 'x.y' where x is major version and y is minor version
 * @type model: Model
 * @ivar model: A Model object representing the neural network used in
 * the archive.
 */

/**
 * The main class of the multi/single-stage model config scheme
 * (multi- stage models consists of interconnected single-stage
 * models).
 *
 * @type config_version: str
 * @ivar config_version: String representing config schema version in
 * format 'x.y' where x is major version and y is minor version
 * @type model: Model
 * @ivar model: A Model object representing the neural network used in
 * the archive.
 */
struct Config {
    /**
     * String representing config schema version in format 'x.y' where x is major version and y
     * is minor version.
     */
    std::optional<std::string> configVersion;
    /**
     * A Model object representing the neural network used in the archive.
     */
    Model model;
};
}  // namespace v1
}  // namespace nn_archive
}  // namespace dai
