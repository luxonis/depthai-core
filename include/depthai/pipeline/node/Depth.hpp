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
 * @brief Composite depth node: picks StereoDepth (RVC2 / RVC3) or NeuralDepth (RVC4) and wires stereo cameras.
 *
 * Add the node to a pipeline, then call build(). The active backend and any auto-created Camera nodes are
 * removed from the pipeline root and adopted as children of this DeviceNodeGroup so IDs and ownership stay
 * consistent. Use depth() and confidence() for queues; use getStereoDepth() / getNeuralDepth() for
 * backend-specific tuning after build().
 */
class Depth : public DeviceNodeGroup {
   public:
    Depth(const Depth&) = delete;
    Depth& operator=(const Depth&) = delete;
    Depth(Depth&&) = delete;
    Depth& operator=(Depth&&) = delete;

    ~Depth() override = default;

    /**
     * @brief Constructs a Depth node bound to the given device (used by Pipeline::create).
     * @param device Non-null device used for platform and stereo-pair discovery.
     * @return Shared Depth instance with buildInternal() already invoked.
     */
    [[nodiscard]] static std::shared_ptr<Depth> create(const std::shared_ptr<Device>& device) {
        auto node = std::shared_ptr<Depth>(new Depth(device));
        node->buildInternal();
        return node;
    }

    /**
     * @brief Creates the platform-specific depth backend and connects stereo ISP outputs.
     *
     * Resolves the device's first stereo pair, finds or creates left/right Camera nodes, requests ISP outputs
     * at 30 FPS, then builds NeuralDepth (RVC4) or StereoDepth (other platforms) and aliases outputs.
     * @param neuralModel Zoo model passed to NeuralDepth::build on RVC4 only; ignored on StereoDepth platforms.
     * @return Shared pointer to this node (same as shared_from_this()).
     * @throws std::runtime_error If build() was already called, there is no device, the node is not in a pipeline,
     *         the device has no stereo pair, or ISP outputs cannot be created.
     */
    std::shared_ptr<Depth> build(DeviceModelZoo neuralModel = DeviceModelZoo::NEURAL_DEPTH_SMALL);

    /**
     * @brief Early initialization hook; Depth performs work in build() instead.
     */
    void buildInternal() override;

    /**
     * @brief Depth frame output of the active backend (StereoDepth::depth or NeuralDepth::depth).
     * @return Reference to the wired Output; valid only after build().
     * @throws std::runtime_error If build() has not been called.
     */
    Node::Output& depth();

    /**
     * @brief Confidence map of the active backend (StereoDepth::confidenceMap or NeuralDepth::confidence).
     * @return Reference to the wired Output; valid only after build().
     * @throws std::runtime_error If build() has not been called.
     */
    Node::Output& confidence();

    /**
     * @brief Access StereoDepth subnode when the platform is not RVC4.
     * @return Non-null after build() on RVC2/RVC3 (and similar); nullptr on RVC4 or before build().
     */
    std::shared_ptr<StereoDepth> getStereoDepth() const {
        return stereo_;
    }

    /**
     * @brief Access NeuralDepth subnode on RVC4.
     * @return Non-null after build() on RVC4; nullptr on other platforms or before build().
     */
    std::shared_ptr<NeuralDepth> getNeuralDepth() const {
        return neural_;
    }

   private:
    /**
     * @brief Device-bound constructor; use create() from application code.
     */
    explicit Depth(const std::shared_ptr<Device>& device);

    /// Stereo backend when platform != RVC4; populated in build().
    std::shared_ptr<StereoDepth> stereo_;
    /// Neural backend on RVC4; populated in build().
    std::shared_ptr<NeuralDepth> neural_;
    /// Points at the active backend's depth Output after build().
    Output* depthOut_{nullptr};
    /// Points at the active backend's confidence Output after build().
    Output* confidenceOut_{nullptr};
    /// True after a successful build(); prevents double build().
    bool built_{false};
};

}  // namespace node
}  // namespace dai
