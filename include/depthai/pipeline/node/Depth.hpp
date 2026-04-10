#pragma once

#include <memory>

#include "depthai/device/Device.hpp"
#include "depthai/pipeline/DeviceNodeGroup.hpp"
#include "depthai/pipeline/node/Camera.hpp"
#include "depthai/pipeline/node/NeuralDepth.hpp"
#include "depthai/pipeline/node/StereoDepth.hpp"
#include "depthai/common/DeviceModelZoo.hpp"

namespace dai {

class Pipeline;

namespace node {

/**
 * @brief Composite depth node: selects StereoDepth (RVC2 / RVC3) or NeuralDepth (RVC4), wires stereo cameras.
 *
 * Call `build()` after adding the node to a pipeline. Subnodes are created at build time (lazy) and adopted
 * under this group. Use `depth()` and `confidence()` to access the active backend outputs.
 */
class Depth : public DeviceNodeGroup {
   public:
    Depth(const Depth&) = delete;
    Depth& operator=(const Depth&) = delete;
    Depth(Depth&&) = delete;
    Depth& operator=(Depth&&) = delete;

    ~Depth() override = default;

    [[nodiscard]] static std::shared_ptr<Depth> create(const std::shared_ptr<Device>& device) {
        auto node = std::shared_ptr<Depth>(new Depth(device));
        node->buildInternal();
        return node;
    }

    /**
     * Creates backend subnode, ensures left/right cameras (or uses existing stereo pair), links ISP outputs at 30 FPS.
     * @param neuralModel Used only on RVC4 (NeuralDepth).
     */
    std::shared_ptr<Depth> build(DeviceModelZoo neuralModel = DeviceModelZoo::NEURAL_DEPTH_SMALL);

    void buildInternal() override;

    /** Depth frame output of the active backend (throws if build() not called). */
    Node::Output& depth();
    /** Confidence output (StereoDepth: confidenceMap; NeuralDepth: confidence). */
    Node::Output& confidence();

    /** Underlying StereoDepth when platform is not RVC4; nullptr after build on RVC4. */
    std::shared_ptr<StereoDepth> getStereoDepth() const {
        return stereo_;
    }
    /** Underlying NeuralDepth on RVC4; nullptr after build on other platforms. */
    std::shared_ptr<NeuralDepth> getNeuralDepth() const {
        return neural_;
    }

   private:
    explicit Depth(const std::shared_ptr<Device>& device);

    std::shared_ptr<StereoDepth> stereo_;
    std::shared_ptr<NeuralDepth> neural_;
    Output* depthOut_{nullptr};
    Output* confidenceOut_{nullptr};
    bool built_{false};
};

}  // namespace node
}  // namespace dai
