#pragma once

#include <depthai/pipeline/DeviceNode.hpp>
#include <depthai/pipeline/Subnode.hpp>
#include <depthai/pipeline/datatype/NeuralDepthConfig.hpp>
#include <depthai/pipeline/node/MessageDemux.hpp>
#include <depthai/pipeline/node/NeuralNetwork.hpp>
#include <depthai/pipeline/node/Rectification.hpp>
#include <depthai/pipeline/node/Sync.hpp>
#include <depthai/properties/NeuralDepthProperties.hpp>
namespace dai {
namespace node {

/**
 * @brief NeuralDepth node. Compute depth from left-right image pair using neural network.
 */
class NeuralDepth : public DeviceNodeCRTP<DeviceNode, NeuralDepth, NeuralDepthProperties> {
   public:
    constexpr static const char* NAME = "NeuralDepth";

   protected:
    Properties& getProperties() override;
    using DeviceNodeCRTP::DeviceNodeCRTP;

    NeuralDepth() = default;
    NeuralDepth(std::unique_ptr<Properties> props);

   public:
    /**
     * Get input size for specific model
     */
    static std::pair<int, int> getInputSize(DeviceModelZoo model);

    /**
     * Enable or disable rectification (useful for prerectified inputs)
     */
    NeuralDepth& setRectification(bool enable);

    /**
     * Initial config to use for NeuralDepth.
     */
    std::shared_ptr<NeuralDepthConfig> initialConfig = std::make_shared<NeuralDepthConfig>();
    std::shared_ptr<NeuralDepth> build(Output& left, Output& right, DeviceModelZoo model = DeviceModelZoo::NEURAL_DEPTH_SMALL);
    Subnode<Sync> sync{*this, "sync"};
    Subnode<MessageDemux> messageDemux{*this, "messageDemux"};
    Subnode<Rectification> rectification{*this, "rectification"};
    Subnode<NeuralNetwork> neuralNetwork{*this, "neuralNetwork"};

#ifndef DEPTHAI_INTERNAL_DEVICE_BUILD_RVC4
    /**
     * Input for left ImgFrame of left-right pair
     */
    Input& left{sync->inputs["left"]};
    /**
     * Input for right ImgFrame of left-right pair
     */
    Input& right{sync->inputs["right"]};

    /**
     * Output for rectified left ImgFrame
     */
    Output& rectifiedLeft{rectification->output1};

    /**
     * Output for rectified right ImgFrame
     */
    Output& rectifiedRight{rectification->output2};
#endif

    /**
     * Input config to modify parameters in runtime.
     */
    Input inputConfig{*this, {"inputConfig", DEFAULT_GROUP, true, 5, {{{DatatypeEnum::NeuralDepthConfig, false}}}}};

    /**
     * Input NNData to parse
     */
    Input nnDataInput{*this, {"nnDataInput", DEFAULT_GROUP, true, 5, {{{DatatypeEnum::NNData, false}}}}};

    /**
     * Input left frame internal, used to extract frame info
     */
    Input leftInternal{*this, {"leftFrameInternal", DEFAULT_GROUP, false, 1, {{{DatatypeEnum::ImgFrame, false}}}}};

    /**
     * Input right frame internal, used to extract frame info
     */
    Input rightInternal{*this, {"rightFrameInternal", DEFAULT_GROUP, false, 1, {{{DatatypeEnum::ImgFrame, false}}}}};

    /**
     * Output disparity ImgFrame
     */
    Output disparity{*this, {"disparity", DEFAULT_GROUP, {{{DatatypeEnum::ImgFrame, false}}}}};

    /**
     * Output depth ImgFrame
     */
    Output depth{*this, {"depth", DEFAULT_GROUP, {{{DatatypeEnum::ImgFrame, false}}}}};

    /**
     * Output edge ImgFrame
     */
    Output edge{*this, {"edge", DEFAULT_GROUP, {{{DatatypeEnum::ImgFrame, false}}}}};

    /**
     * Output confidence ImgFrame
     */
    Output confidence{*this, {"confidence", DEFAULT_GROUP, {{{DatatypeEnum::ImgFrame, false}}}}};

    void buildInternal() override;
};

}  // namespace node
}  // namespace dai
