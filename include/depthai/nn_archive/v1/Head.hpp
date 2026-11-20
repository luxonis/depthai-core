#pragma once

#include <optional>
#include <string>
#include <utility>
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

/** @brief Read Head from its NNArchive JSON representation. */
inline void from_json(const nlohmann::json& j, Head& value) {
    Head head;
    j.at("metadata").get_to(head.metadata);
    if(j.contains("name")) j.at("name").get_to(head.name);
    if(j.contains("outputs")) j.at("outputs").get_to(head.outputs);
    j.at("parser").get_to(head.parser);
    value = std::move(head);
}

/** @brief Write Head in NNArchive JSON format, preserving custom metadata fields. */
inline void to_json(nlohmann::json& j, const Head& value) {
    j = nlohmann::json::object();
    j["metadata"] = value.metadata;
    j["name"] = value.name;
    j["outputs"] = value.outputs;
    j["parser"] = value.parser;
}

NOP_EXTERNAL_STRUCTURE(Head, metadata, name, outputs, parser);

}  // namespace v1
}  // namespace nn_archive
}  // namespace dai
