#pragma once

#include <depthai/pipeline/DeviceNode.hpp>
#include <depthai/pipeline/Subnode.hpp>
#include <depthai/pipeline/datatype/NeuralAssistedStereoV2Config.hpp>
#include <depthai/pipeline/node/MessageDemux.hpp>
#include <depthai/pipeline/node/Rectification.hpp>
#include <depthai/pipeline/node/Sync.hpp>
#include <depthai/properties/NeuralAssistedStereoV2Properties.hpp>

namespace dai {
namespace node {

class NeuralAssistedStereoV2 : public DeviceNodeCRTP<DeviceNode, NeuralAssistedStereoV2, NeuralAssistedStereoV2Properties> {
   protected:
    using DeviceNodeCRTP::DeviceNodeCRTP;
    Properties& getProperties() override;
    NeuralAssistedStereoV2(std::unique_ptr<Properties> props);

   public:
    constexpr static const char* NAME = "NeuralAssistedStereoV2";
    NeuralAssistedStereoV2() = default;

    std::shared_ptr<NeuralAssistedStereoV2Config> initialConfig = std::make_shared<NeuralAssistedStereoV2Config>();

    std::shared_ptr<NeuralAssistedStereoV2> build(Output& left, Output& right);
    NeuralAssistedStereoV2& setRectification(bool enable);

    Subnode<Sync> sync{*this, "sync"};
    Subnode<MessageDemux> messageDemux{*this, "messageDemux"};
    Subnode<Rectification> rectification{*this, "rectification"};

#ifndef DEPTHAI_INTERNAL_DEVICE_BUILD_RVC4
    Input& left{sync->inputs["left"]};
    Input& right{sync->inputs["right"]};

    Output& rectifiedLeft{rectification->output1};
    Output& rectifiedRight{rectification->output2};
#endif

    /// Config input (NeuralAssistedStereoV2Config). Optional, updates at runtime.
    Input inputConfig{*this, {"inputConfig", DEFAULT_GROUP, true, 5, {{{DatatypeEnum::NeuralAssistedStereoV2Config, false}}}}};

    /// Neural disparity prior input (RAW16, any resolution — upsampled to full-res internally).
    /// Pixel values are integer disparity (no subpixel scaling).
    Input neuralDisparity{*this, {"neuralDisparity", DEFAULT_GROUP, true, 5, {{{DatatypeEnum::ImgFrame, false}}}}};

    Input leftInternal{*this, {"leftFrameInternal", DEFAULT_GROUP, false, 1, {{{DatatypeEnum::ImgFrame, false}}}}};
    Input rightInternal{*this, {"rightFrameInternal", DEFAULT_GROUP, false, 1, {{{DatatypeEnum::ImgFrame, false}}}}};

    Output disparity{*this, {"disparity", DEFAULT_GROUP, {{{DatatypeEnum::ImgFrame, false}}}}};
    Output depth{*this, {"depth", DEFAULT_GROUP, {{{DatatypeEnum::ImgFrame, false}}}}};
    Output debugZnccCurve{*this, {"debugZnccCurve", DEFAULT_GROUP, {{{DatatypeEnum::ImgFrame, false}}}}};

    void buildInternal() override;
};

}  // namespace node
}  // namespace dai
