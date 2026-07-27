#pragma once

#include "depthai/pipeline/ThreadedHostNode.hpp"
#include "depthai/pipeline/datatype/ImgDetections.hpp"

namespace dai {
namespace beta {
namespace node {

/**
 * @brief Experimental host node for filtering image detections.
 *
 * This initial implementation forwards ImgDetections messages unchanged.
 */
class ImgDetectionsFilter : public NodeCRTP<dai::node::ThreadedHostNode, ImgDetectionsFilter> {
   public:
    constexpr static const char* NAME = "ImgDetectionsFilter";

    Input input{*this, {"input", DEFAULT_GROUP, DEFAULT_BLOCKING, DEFAULT_QUEUE_SIZE, {{{DatatypeEnum::ImgDetections, false}}}, DEFAULT_WAIT_FOR_MESSAGE}};
    Output output{*this, {"output", DEFAULT_GROUP, {{{DatatypeEnum::ImgDetections, false}}}}};

    void run() override;
};

}  // namespace node
}  // namespace beta
}  // namespace dai
