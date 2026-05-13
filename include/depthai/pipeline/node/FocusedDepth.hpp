#pragma once

#include <memory>

#include "depthai/common/DeviceModelZoo.hpp"
#include "depthai/device/Device.hpp"
#include "depthai/pipeline/DeviceNodeGroup.hpp"
#include "depthai/pipeline/Subnode.hpp"
#include "depthai/pipeline/datatype/StereoDepthConfig.hpp"
#include "depthai/pipeline/node/FocusedDepthCrop.hpp"
#include "depthai/pipeline/node/NeuralDepth.hpp"
#include "depthai/pipeline/node/StereoDepth.hpp"
#include "depthai/pipeline/node/Sync.hpp"

namespace dai {

namespace node {

/**
 * @brief RVC4-focused depth on a dynamic ROI: synchronizes left/right/FocusedDepthRoi, crops with disparity margin on host,
 * then runs either StereoDepth or NeuralDepth (session-stable backend).
 */
class FocusedDepth : public DeviceNodeGroup {
   public:
    enum class Backend : std::uint32_t { AUTO = 0, STEREO, NEURAL };

    FocusedDepth(const FocusedDepth&) = delete;
    FocusedDepth& operator=(const FocusedDepth&) = delete;
    FocusedDepth(FocusedDepth&&) = delete;
    FocusedDepth& operator=(FocusedDepth&&) = delete;

    ~FocusedDepth() override = default;

    [[nodiscard]] static std::shared_ptr<FocusedDepth> create(const std::shared_ptr<Device>& device) {
        return std::shared_ptr<FocusedDepth>(new FocusedDepth(device));
    }

    /// Smallest NeuralDepth zoo model whose input resolution is at least \p width x \p height (letterbox target).
    [[nodiscard]] static DeviceModelZoo pickNeuralModelForRoi(int width, int height);

    std::shared_ptr<FocusedDepth> setBackend(Backend backend) {
        backend_ = backend;
        return std::static_pointer_cast<FocusedDepth>(shared_from_this());
    }

    [[nodiscard]] Backend getBackend() const {
        return backend_;
    }

    std::shared_ptr<FocusedDepth> setTargetFps(float fps) {
        targetFps_ = fps;
        return std::static_pointer_cast<FocusedDepth>(shared_from_this());
    }

    std::shared_ptr<FocusedDepth> setMaxRoi(uint32_t width, uint32_t height) {
        maxRoiWidth_ = width;
        maxRoiHeight_ = height;
        return std::static_pointer_cast<FocusedDepth>(shared_from_this());
    }

    std::shared_ptr<FocusedDepth> setNeuralModelOverride(DeviceModelZoo model) {
        neuralModelOverride_ = model;
        hasNeuralModelOverride_ = true;
        return std::static_pointer_cast<FocusedDepth>(shared_from_this());
    }

    void buildInternal() override;

    Node::Output& depth();
    Node::Output& confidence();

    StereoDepth* getStereoDepth() const {
        return stereoBackend_ ? &**stereoBackend_ : nullptr;
    }

    NeuralDepth* getNeuralDepth() const {
        return neuralBackend_ ? &**neuralBackend_ : nullptr;
    }

    std::shared_ptr<StereoDepthConfig> stereoInitialConfig = std::make_shared<StereoDepthConfig>();

    Subnode<Sync> sync{*this, "focusedDepthSync"};
    Input& left{sync->inputs["left"]};
    Input& right{sync->inputs["right"]};
    Input& roi{sync->inputs["roi"]};

   private:
    explicit FocusedDepth(const std::shared_ptr<Device>& device);

    Backend resolveSessionBackend() const;

    Backend backend_{Backend::AUTO};
    float targetFps_{30.F};
    uint32_t maxRoiWidth_{640};
    uint32_t maxRoiHeight_{400};
    DeviceModelZoo neuralModelOverride_{DeviceModelZoo::NEURAL_DEPTH_SMALL};
    bool hasNeuralModelOverride_{false};

    Subnode<FocusedDepthCrop> crop_{*this, "focusedDepthCrop"};
    std::unique_ptr<Subnode<StereoDepth>> stereoBackend_;
    std::unique_ptr<Subnode<NeuralDepth>> neuralBackend_;

    bool graphBuilt_{false};
    Node::Output* depthOut_{nullptr};
    Node::Output* confidenceOut_{nullptr};
};

}  // namespace node
}  // namespace dai
