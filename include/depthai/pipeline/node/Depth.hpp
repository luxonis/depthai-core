#pragma once

#include <memory>

#include "depthai/common/DeviceModelZoo.hpp"
#include "depthai/common/StereoPair.hpp"
#include "depthai/device/Device.hpp"
#include "depthai/pipeline/DeviceNodeGroup.hpp"
#include "depthai/pipeline/Subnode.hpp"
#include "depthai/pipeline/node/Camera.hpp"
#include "depthai/pipeline/node/NeuralDepth.hpp"
#include "depthai/pipeline/node/StereoDepth.hpp"

namespace dai {

class Pipeline;

namespace node {

/**
 * @brief Composite depth node: StereoDepth (non-RVC4) or NeuralDepth (RVC4) as ``Subnode`` children.
 *
 * The active backend ``Subnode`` is created in the constructor so it is present when the pipeline registers
 * ``Depth`` and assigns pipeline parents to the whole subtree. Graph wiring (cameras, ``requestOutput``, backend
 * ``build``) runs inside ``buildInternal()``, which is invoked lazily the first time ``depth()`` or ``confidence()``
 * is used once ``Depth`` is on a pipeline (``parent`` set). Missing ``Camera`` nodes are created with
 * ``Pipeline::create`` (top-level nodes, not adopted). Existing pair cameras stay on the pipeline and only get an
 * extra ``requestOutput``.
 *
 * ``depth()`` / ``confidence()`` refer to backend outputs immediately; create host queues **before**
 * ``pipeline.build()`` / ``start()``. ``build()`` is a no-op for API compatibility. On RVC4 the NeuralDepth zoo
 * model is fixed inside ``buildInternal()`` (not configurable via ``build()``).
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
     */
    [[nodiscard]] static std::shared_ptr<Depth> create(const std::shared_ptr<Device>& device) {
        return std::shared_ptr<Depth>(new Depth(device));
    }

    /**
     * @brief No-op (returns ``shared_from_this()``). Kept for example/script compatibility; ``neuralModel`` is ignored.
     */
    std::shared_ptr<Depth> build(DeviceModelZoo /*neuralModel*/ = DeviceModelZoo::NEURAL_DEPTH_SMALL) {
        return std::static_pointer_cast<Depth>(shared_from_this());
    }

    void buildInternal() override;

    /// Backend depth output; first access may run ``buildInternal()`` wiring if not done yet.
    Node::Output& depth();
    /// Backend confidence output; first access may run ``buildInternal()`` wiring if not done yet.
    Node::Output& confidence();

    StereoDepth* getStereoDepth() const {
        return stereoBackend_ ? &**stereoBackend_ : nullptr;
    }

    NeuralDepth* getNeuralDepth() const {
        return neuralBackend_ ? &**neuralBackend_ : nullptr;
    }

   private:
    explicit Depth(const std::shared_ptr<Device>& device);

    std::pair<Node::Output*, Node::Output*> ensureStereoCameraOutputs(Pipeline& pipeline,
                                                                       const StereoPair& pair,
                                                                       std::pair<uint32_t, uint32_t> frameSize,
                                                                       float monoFps);

    std::unique_ptr<::dai::Subnode<StereoDepth>> stereoBackend_;
    std::unique_ptr<::dai::Subnode<NeuralDepth>> neuralBackend_;

    bool graphBuilt_{false};

    Output* depthOut_{nullptr};
    Output* confidenceOut_{nullptr};
};

}  // namespace node
}  // namespace dai
