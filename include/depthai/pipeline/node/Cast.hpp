#pragma once

#include <depthai/pipeline/Node.hpp>

// shared
#include <depthai-shared/properties/CastProperties.hpp>

namespace dai {
namespace node {

/**
 * @brief Cast node.
 */
class Cast : public NodeCRTP<Node, Cast, CastProperties> {
   public:
    constexpr static const char* NAME = "Cast";

    Cast(const std::shared_ptr<PipelineImpl>& par, int64_t nodeId);
    Cast(const std::shared_ptr<PipelineImpl>& par, int64_t nodeId, std::unique_ptr<Properties> props);

    /**
     * Input NNData or ImgFrame message.
     */
    Input input{*this, "input", Input::Type::SReceiver, false, 4, true, {{DatatypeEnum::ImgFrame, false}, {DatatypeEnum::NNData, false}}};

    /**
     * Output ImgFrame message.
     */
    Output output{*this, "output", Output::Type::MSender, {{DatatypeEnum::ImgFrame, false}}};

    /**
     * Passthrough input message.
     */
    Output passthroughInput{*this, "passthroughInput", Output::Type::MSender, {{DatatypeEnum::ImgFrame, false}, {DatatypeEnum::NNData, false}}};

    /**
     * Set number of frames in pool
     * @param numFramesPool Number of frames in pool
     */
    Cast& setNumFramesPool(int numFramesPool);

    /**
     * Set output frame type
     * @param outputType Output frame type
     */
    Cast& setOutputFrameType(dai::RawImgFrame::Type outputType);

    /**
     * Set scale
     * @param scale Scale
     */
    Cast& setScale(float scale);

    /**
     * Set offset
     * @param offset Offset
     */
    Cast& setOffset(float offset);
};

}  // namespace node
}  // namespace dai
