#pragma once

#include <depthai/common/DeviceModelZoo.hpp>
#include <depthai/pipeline/datatype/ImgDetections.hpp>
#include <depthai/pipeline/datatype/ImgFrame.hpp>
#include <depthai/pipeline/datatype/MessageGroup.hpp>
#include <depthai/pipeline/node/Sync.hpp>
#include <depthai/pipeline/node/host/HostNode.hpp>

#include <memory>
#include <optional>
#include <utility>

namespace dai {
namespace node {

class FocusController : public CustomNode<FocusController> {
   public:
    FocusController() = default;

    std::shared_ptr<FocusController> build(float targetFps = 30.0f);

    Input depthCrop{*this, {"depthCrop", DEFAULT_GROUP, DEFAULT_BLOCKING, DEFAULT_QUEUE_SIZE, {{{DatatypeEnum::Buffer, true}}}, DEFAULT_WAIT_FOR_MESSAGE}};
    Input confidenceCrop{*this,
                         {"confidenceCrop", DEFAULT_GROUP, DEFAULT_BLOCKING, DEFAULT_QUEUE_SIZE, {{{DatatypeEnum::Buffer, true}}}, DEFAULT_WAIT_FOR_MESSAGE}};

    Output leftConfig{*this, {"leftConfig", DEFAULT_GROUP, {{{DatatypeEnum::ImageManipConfig, true}}}}};
    Output rightConfig{*this, {"rightConfig", DEFAULT_GROUP, {{{DatatypeEnum::ImageManipConfig, true}}}}};

    Output gateControlNeural{*this, {"gateControlNeural", DEFAULT_GROUP, {{{DatatypeEnum::GateControl, true}}}}};
    Output gateControlStereo{*this, {"gateControlStereo", DEFAULT_GROUP, {{{DatatypeEnum::GateControl, true}}}}};
    Output gateControlGpu{*this, {"gateControlGpu", DEFAULT_GROUP, {{{DatatypeEnum::GateControl, true}}}}};

    Output confidenceOut{*this, {"confidenceOut", DEFAULT_GROUP, {{{DatatypeEnum::Buffer, true}}}}};

    std::shared_ptr<Buffer> processGroup(std::shared_ptr<MessageGroup> in) override;

    void setTargetFps(float targetFps);
    void setNeuralModel(DeviceModelZoo model);
    void setStereoSize(int width, int height);
    void setStereoFps(float fps);

    constexpr static const char* NAME = "FocusController";

   private:
    enum class Backend { NEURAL, STEREO, GPU };

    float targetFps_ = 30.0f;
    DeviceModelZoo neuralModel_ = DeviceModelZoo::NEURAL_DEPTH_480X300;
    std::pair<int, int> neuralSize_ = {480, 300};
    float neuralFps_ = 56.0f;
    std::pair<int, int> stereoSize_ = {640, 400};
    float stereoFps_ = 30.0f;
};

}  // namespace node
}  // namespace dai
