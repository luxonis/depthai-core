#pragma once

#include <depthai/pipeline/DeviceNode.hpp>

// standard
#include <fstream>

// shared
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
 * @brief NeuralDepth node. Computes point cloud from depth frames.
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

   private:
    std::unordered_map<DeviceModelZoo, std::pair<int, int>> modelToInputSize = {
        {DeviceModelZoo::NEURAL_DEPTH_LARGE, {768, 480}},
        {DeviceModelZoo::NEURAL_DEPTH_MEDIUM, {576, 360}},
        {DeviceModelZoo::NEURAL_DEPTH_SMALL, {480, 300}},
        {DeviceModelZoo::NEURAL_DEPTH_NANO, {384, 240}},
    };
};

}  // namespace node
}  // namespace dai
