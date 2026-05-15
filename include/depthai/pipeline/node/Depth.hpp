#pragma once

#include <memory>
#include <optional>

#include "depthai/common/StereoPair.hpp"
#include "depthai/device/Device.hpp"
#include "depthai/pipeline/DeviceNodeGroup.hpp"
#include "depthai/pipeline/Subnode.hpp"
#include "depthai/pipeline/node/Camera.hpp"
#include "depthai/pipeline/node/GPUStereo.hpp"
#include "depthai/pipeline/node/NeuralAssistedStereo.hpp"
#include "depthai/pipeline/node/NeuralDepth.hpp"
#include "depthai/pipeline/node/StereoDepth.hpp"
#include "depthai/pipeline/node/ToF.hpp"

namespace dai {

class Pipeline;

namespace node {

/**
 * @brief Composite depth node: StereoDepth, NeuralDepth, NeuralAssistedStereo, ToF, or GPUStereo.
 *
 * ``Algorithm::AUTO`` uses NeuralDepth on RVC4, ToF on RVC2 when a ToF sensor is reported in ``getConnectedCameraFeatures()``,
 * and ``StereoDepth`` on other combinations (including RVC3 and RVC2 without ToF).
 * Use ``Depth()`` / ``Depth::create()`` for automatic algorithm selection, ``explicit Depth(Algorithm)`` / ``create(Algorithm)`` / ``create(device, Algorithm)`` to fix the backend, or ``Pipeline::create<Depth>(…)`` / ``Pipeline::create<Depth>(algorithm)`` to add the node with the pipeline default device. The algorithm is fixed at construction; it cannot be changed after the node is created.
 * ``confidence()`` maps to backend confidence when present; for GPUStereo it uses
 * ``disparity``; for ToF it uses ``amplitude``. Backend implementation nodes are internal and wired with fixed defaults;
 * only ``Algorithm`` selection is configurable at construction (``Depth(...)`` / ``create(...)`` / ``Pipeline::create<Depth>(...)``).
 *
 * Algorithm availability: ``TOF`` requires a connected ToF sensor (see ``Device::getConnectedCameraFeatures()``).
 * ``NEURAL_ASSISTED_STEREO`` is RVC4-only. ``GPU_STEREO`` is RVC4-only, requires a Kompute-enabled build, and excludes Lite-class SKUs
 * (heuristic on product name); extend checks as hardware matrix evolves.
 *
 * See \ref depth_node for a dedicated overview of purpose, behavior, features, and constraints.
 */
class Depth : public DeviceNodeGroup {
   public:
    enum class Algorithm : std::uint32_t {
        AUTO = 0,
        STEREO,
        NEURAL,
        NEURAL_ASSISTED_STEREO,
        TOF,
        GPU_STEREO,
    };

    Depth(const Depth&) = delete;
    Depth& operator=(const Depth&) = delete;
    Depth(Depth&&) = delete;
    Depth& operator=(Depth&&) = delete;

    ~Depth() override = default;

    /** Default ``Algorithm::AUTO`` (resolved when the graph is wired). */
    Depth();
    explicit Depth(Algorithm algorithm);

    /** Create Depth with ``Algorithm::AUTO`` and no device bound yet (pipeline default applies when added or at first wiring). */
    [[nodiscard]] static std::shared_ptr<Depth> create() {
        return std::shared_ptr<Depth>(new Depth());
    }

    /** Create Depth. If \p device is null, the device is taken from the pipeline default when the node is added to a pipeline, or from ``getParentPipeline().getDefaultDevice()`` when the graph is first wired. */
    [[nodiscard]] static std::shared_ptr<Depth> create(const std::shared_ptr<Device>& device, Algorithm algorithm = Algorithm::AUTO) {
        auto ptr = std::shared_ptr<Depth>(new Depth(algorithm));
        if(device != nullptr) {
            ptr->setDevice(device);
        }
        return ptr;
    }

    /** Equivalent to ``create(nullptr, algorithm)``. */
    [[nodiscard]] static std::shared_ptr<Depth> create(Algorithm algorithm) {
        return create(nullptr, algorithm);
    }

    /**
     * Set requested FPS for stereo camera outputs used by stereo-based backends; ``nullopt`` uses Camera default.
     * Must be called before the first ``depth()`` / ``confidence()`` access (lazy wiring). Not applied after the graph is built.
     */
    std::shared_ptr<Depth> build(std::optional<float> fps = std::nullopt);

    /** Current algorithm selection (including AUTO), fixed at construction. */
    [[nodiscard]] Algorithm getAlgorithm() const {
        return algorithmOverride_;
    }

    void buildInternal() override;

    Node::Output& depth();
    Node::Output& confidence();

   private:
    Algorithm resolveAlgorithm(const std::shared_ptr<Device>& device) const;
    void validateAlgorithm(const std::shared_ptr<Device>& device, Algorithm active) const;

    /** Ensures stereo cameras exist and returns left/right outputs. Full resolution when @p frameSize is nullopt. */
    std::pair<Node::Output*, Node::Output*> ensureStereoOutputs(Pipeline& pipeline,
                                                                const StereoPair& pair,
                                                                std::optional<std::pair<uint32_t, uint32_t>> frameSize,
                                                                const std::optional<float>& fps);

    Algorithm algorithmOverride_;
    std::optional<float> stereoOutputFps_{};

    std::unique_ptr<::dai::Subnode<StereoDepth>> stereoBackend_;
    std::unique_ptr<::dai::Subnode<NeuralDepth>> neuralBackend_;
    std::unique_ptr<::dai::Subnode<GPUStereo>> gpuStereoBackend_;
    std::shared_ptr<NeuralAssistedStereo> nasBackend_;
    std::shared_ptr<ToF> tofBackend_;

    bool graphBuilt_{false};
    Node::Output* depthOut_{nullptr};
    Node::Output* confidenceOut_{nullptr};
};

}  // namespace node
}  // namespace dai
