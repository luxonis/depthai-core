#pragma once

#include <optional>
#include <string>
#include <vector>

#include "Metadata.hpp"

namespace dai {
namespace nn_archive {
namespace v1 {
/**
 * Represents head of a model.
 *
 * @type name: str | None
 * @ivar name: Optional name of the head.
 * @type parser: str
 * @ivar parser: Name of the parser responsible for processing the
 * models output.
 * @type outputs: List[str] | None
 * @ivar outputs: Specify which outputs are fed into the parser. If
 * None, all outputs are fed.
 * @type metadata: C{HeadMetadata} | C{HeadObjectDetectionMetadata} |
 * C{HeadClassificationMetadata} |
 * C{HeadObjectDetectionSSDMetadata} | C{HeadSegmentationMetadata}
 * | C{HeadYOLOMetadata}
 * @ivar metadata: Metadata of the parser.
 */

/**
 * Represents head of a model.
 *
 * @type name: str | None
 * @ivar name: Optional name of the head.
 * @type parser: str
 * @ivar parser: Name of the parser responsible for processing the
 * models output.
 * @type outputs: List[str] | None
 * @ivar outputs: Specify which outputs are fed into the parser. If
 * None, all outputs are fed.
 * @type metadata: C{HeadMetadata} | C{HeadObjectDetectionMetadata} |
 * C{HeadClassificationMetadata} |
 * C{HeadObjectDetectionSSDMetadata} | C{HeadSegmentationMetadata}
 * | C{HeadYOLOMetadata}
 * @ivar metadata: Metadata of the parser.
 */
struct Head {
    /**
     * Metadata of the parser.
     */
    Metadata metadata;
    /**
     * Optional name of the head.
     */
    std::optional<std::string> name;
    /**
     * Specify which outputs are fed into the parser. If None, all outputs are fed.
     */
    std::optional<std::vector<std::string>> outputs;
    /**
     * Name of the parser responsible for processing the models output.
     */
    std::string parser;
};
}  // namespace v1
}  // namespace nn_archive
}  // namespace dai
