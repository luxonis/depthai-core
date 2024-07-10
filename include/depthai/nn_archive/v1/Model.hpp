#pragma once

#include <vector>
#include <optional>

#include "Head.hpp"
#include "Input.hpp"
#include "MetadataClass.hpp"
#include "Output.hpp"

namespace dai {
namespace nn_archive {
namespace v1 {
    /**
     * A Model object representing the neural network used in the archive.
     *
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


    /**
     * A Model object representing the neural network used in the archive.
     *
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
        std::optional<std::vector<Head>> heads;
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
}
