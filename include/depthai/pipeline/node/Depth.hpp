#pragma once

#include <memory>
#include <optional>
#include <vector>

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
 * Use ``Depth()`` / ``Depth::create()`` for automatic algorithm selection, ``explicit Depth(Algorithm)`` / ``create(Algorithm)`` / ``create(device, Algorithm)`` to fix the backend, or ``Pipeline::create<Depth>(…)`` / ``Pipeline::create<Depth>(algorithm)`` to add the node with the pipeline default device. The algorithm can also be set before first wiring through ``build(algorithm, ...)``.
 * ``confidence()`` maps to backend confidence when present. For GPUStereo (``Algorithm::GPU_STEREO``) it is unavailable
 * (throws); use ``hasConfidence()`` before linking or creating a queue. For ToF it uses ``amplitude``.
 * Backend implementation nodes are internal and wired with fixed defaults;
 * only ``Algorithm`` selection is user-configurable, either at construction or through ``build(algorithm, ...)`` before first wiring.
 *
 * Algorithm availability: ``TOF`` requires a connected ToF sensor (see ``Device::getConnectedCameraFeatures()``).
 * ``NEURAL_ASSISTED_STEREO`` is RVC4-only. ``GPU_STEREO`` is RVC4-only, requires a stereo pair and board revision 9 or newer
 * (EEPROM ``boardRev`` e.g. ``P9D1``, ``P10D0``, or legacy ``R9``); it is listed in ``getSupportedAlgorithms()`` only when those
 * requirements are met.
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

    /**
     * Create Depth with ``Algorithm::AUTO`` and no device bound yet.
     * Pipeline default device applies when added or at first wiring.
     */
    [[nodiscard]] static std::shared_ptr<Depth> create() {
        return std::shared_ptr<Depth>(new Depth());
    }

    /**
     * Create Depth with an optional fixed device.
     * If \p device is null, the pipeline default device is used when the node is added.
     */
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

    /**
     * Set algorithm and optional stereo camera FPS before first wiring.
     * The algorithm must not be changed after the graph is built.
     */
    std::shared_ptr<Depth> build(Algorithm algorithm, std::optional<float> fps = std::nullopt);

    /** Current requested algorithm selection (including ``AUTO``). */
    [[nodiscard]] Algorithm getAlgorithm() const {
        return algorithmOverride_;
    }

    /** Returns the stereo pair used by stereo-based Depth backends on the current device. */
    [[nodiscard]] StereoPair getStereoPair() const;

    /** Returns algorithms supported by the supplied device. */
    std::vector<Algorithm> getSupportedAlgorithms(const std::shared_ptr<Device>& device) const;

    void buildInternal() override;

    Node::Output& depth();
    Node::Output& confidence();

    /** False when the resolved backend has no confidence map (GPUStereo). Triggers lazy wiring if needed. */
    [[nodiscard]] bool hasConfidence() const;

   private:
    Algorithm selectAlgorithm(const std::shared_ptr<Device>& device) const;

    /** Ensures stereo cameras exist and returns left/right outputs. Full resolution when @p frameSize is nullopt. */
    std::pair<Node::Output*, Node::Output*> ensureStereoOutputs(Pipeline& pipeline,
                                                                const StereoPair& pair,
                                                                std::optional<std::pair<uint32_t, uint32_t>> frameSize,
                                                                const std::optional<float>& fps);

    Algorithm algorithmOverride_;
    bool graphBuilt_{false};
    std::optional<float> stereoOutputFps_{};

    Node::Output* depthOut_{nullptr};
    Node::Output* confidenceOut_{nullptr};

    std::unique_ptr<::dai::Subnode<StereoDepth>> stereoBackend_;
    std::unique_ptr<::dai::Subnode<NeuralDepth>> neuralBackend_;
    std::unique_ptr<::dai::Subnode<GPUStereo>> gpuStereoBackend_;
    std::shared_ptr<NeuralAssistedStereo> nasBackend_;
    std::shared_ptr<ToF> tofBackend_;
};

}  // namespace node
}  // namespace dai
