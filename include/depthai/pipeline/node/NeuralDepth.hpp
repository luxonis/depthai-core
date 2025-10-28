#pragma once

#include <depthai/pipeline/DeviceNode.hpp>

// standard
#include <fstream>

// shared
#include <depthai/properties/NeuralDepthProperties.hpp>
#include <memory>

namespace dai {
namespace node {

/**
 * @brief NeuralDepth node. Computes point cloud from depth frames.
 */
class NeuralDepth : public DeviceNodeCRTP<DeviceNode, NeuralDepth, NeuralDepthProperties> {
   public:
    constexpr static const char* NAME = "NeuralDepth";

   protected:
    Properties& getProperties() override;
    using DeviceNodeCRTP::DeviceNodeCRTP;

   public:

    /**
     * Input for left ImgFrame of left-right pair
     */
    Input left{*this, {"left", DEFAULT_GROUP, DEFAULT_BLOCKING, DEFAULT_QUEUE_SIZE, {{{DatatypeEnum::ImgFrame, true}}}, DEFAULT_WAIT_FOR_MESSAGE}};

    /**
     * Input for right ImgFrame of left-right pair
     */
    Input right{*this, {"right", DEFAULT_GROUP, DEFAULT_BLOCKING, DEFAULT_QUEUE_SIZE, {{{DatatypeEnum::ImgFrame, true}}}, DEFAULT_WAIT_FOR_MESSAGE}};


    Output disparity{*this, {"disparity", DEFAULT_GROUP, {{{DatatypeEnum::ImgFrame, false}}}}};
    Output depth{*this, {"depth", DEFAULT_GROUP, {{{DatatypeEnum::ImgFrame, false}}}}};
    Output edge{*this, {"edge", DEFAULT_GROUP, {{{DatatypeEnum::ImgFrame, false}}}}};
    Output confidence{*this, {"confidence", DEFAULT_GROUP, {{{DatatypeEnum::ImgFrame, false}}}}};


    NeuralDepth& setModelType(NeuralDepthProperties::ModelType modelType);

    NeuralDepthProperties::ModelType getModelType() const;
};

}  // namespace node
}  // namespace dai
