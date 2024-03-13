#pragma once

#include <optional>

namespace dai {
namespace nn_archive_v1 {
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
    struct Metadata {
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
