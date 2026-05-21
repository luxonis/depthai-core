#pragma once

#include <memory>
#include <optional>
#include <utility>
#include <vector>

#include "depthai/common/DeviceModelZoo.hpp"
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
 * On RVC4 ``Algorithm::AUTO`` picks the algorithm based on user resolution and target FPS,
 * preferring NeuralDepth → NeuralAssistedStereo → StereoDepth → GPUStereo. The algorithm-specific
 * "profile" is selected separately: NeuralDepth maps to a ``DeviceModelZoo`` model, StereoDepth maps
 * to a ``StereoDepth::PresetMode``; NeuralAssistedStereo and GPUStereo have no selectable profile.
 * On other platforms, AUTO uses ToF (RVC2 with ToF sensor) or ``StereoDepth``.
 *
 * ``confidence()`` maps to backend confidence when present. For GPUStereo (``Algorithm::GPU_STEREO``) it is unavailable
 * (throws); use ``hasConfidence()`` first. For ToF it uses ``amplitude``.
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
    /** Backend selection for the composite Depth node. */
    enum class Algorithm : std::uint32_t {
        AUTO = 0,  ///< Resolved at wiring via device capabilities and stereo size / FPS (RVC4).
        STEREO,    ///< Classic StereoDepth disparity/depth pipeline.
        NEURAL,    ///< RVC4 NeuralDepth; model auto-selected on RVC4 unless overridden.
        NEURAL_ASSISTED_STEREO,  ///< RVC4 NeuralAssistedStereo (stereo + neural assist).
        TOF,       ///< Time-of-flight depth when a ToF sensor is connected.
        GPU_STEREO,  ///< RVC4 GPUStereo (no confidence output).
    };

    // Non-copyable / non-movable: owns pipeline subnodes and output pointers tied to this instance.
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

    /**
     * Set algorithm, optional FPS, and stereo frame size used by RVC4 auto-selection
     * when no upstream ``Camera`` is present.
     */
    std::shared_ptr<Depth> build(Algorithm algorithm,
                                  std::optional<float> fps,
                                  std::optional<std::pair<uint32_t, uint32_t>> stereoSize);

    /** Pin the NeuralDepth zoo model (skips RVC4 auto model picker). */
    std::shared_ptr<Depth> build(DeviceModelZoo neuralModel);

    /** Current requested algorithm selection (including ``AUTO``). */
    [[nodiscard]] Algorithm getAlgorithm() const {
        return algorithmOverride_;
    }

    /** Algorithm actually wired (``AUTO`` resolved). Valid after first ``depth()`` access. */
    [[nodiscard]] Algorithm getResolvedAlgorithm() const {
        return resolvedAlgorithm_;
    }

    /**
     * NeuralDepth zoo model used by the resolved backend.
     * For ``NEURAL`` this is the auto-selected (or overridden) model.
     * For ``NEURAL_ASSISTED_STEREO`` this is always ``NEURAL_DEPTH_NANO`` (NAS has no selectable profile).
     * For other backends the value is unspecified.
     */
    [[nodiscard]] DeviceModelZoo getResolvedNeuralModel() const {
        return resolvedNeuralModel_;
    }

    /** ``StereoDepth`` preset used when the resolved backend is ``STEREO``. */
    [[nodiscard]] StereoDepth::PresetMode getResolvedStereoPreset() const {
        return resolvedStereoPreset_;
    }

    /** Returns algorithms supported by the supplied device. */
    std::vector<Algorithm> getSupportedAlgorithms(const std::shared_ptr<Device>& device) const;

    /** True when @p width or @p height exceeds ``StereoDepth`` RVC4 maximum (1280x1280). */
    static bool exceedsStereoDepthMaxResolution(uint32_t width, uint32_t height);

    /** RVC4 NeuralDepth model picker (quality-first under FPS budget; ``supportedModels`` empty = all). */
    static DeviceModelZoo selectNeuralDepthModel(uint32_t userWidth,
                                                  uint32_t userHeight,
                                                  float targetFps,
                                                  const std::vector<DeviceModelZoo>& supportedModels = {});

    /** RVC4 ``StereoDepth`` preset picker (quality-first under FPS budget). */
    static StereoDepth::PresetMode selectStereoDepthPreset(float targetFps);

    /** Lazily wires the selected backend subgraph when the node is attached to a pipeline. */
    void buildInternal() override;

    /** Depth map from the active backend; triggers ``buildInternal()`` on first access. */
    Node::Output& depth();
    /**
     * Confidence map from the active backend (StereoDepth confidence, NeuralDepth confidence, ToF amplitude).
     * Throws if the backend has no confidence output (GPUStereo); use ``hasConfidence()`` first.
     */
    Node::Output& confidence();

    /** False when the resolved backend has no confidence map (GPUStereo). Triggers lazy wiring if needed. */
    [[nodiscard]] bool hasConfidence() const;

   private:
    /** Resolve ``AUTO`` or validate a fixed ``algorithmOverride_`` against @p device. */
    Algorithm selectAlgorithm(const std::shared_ptr<Device>& device) const;

    /** Resolve final algorithm + neural models, considering RVC4 AUTO size/FPS heuristics. */
    void resolveWiring(const std::shared_ptr<Device>& device, Pipeline& pipeline);

    /**
     * Ensures stereo cameras exist and returns left/right outputs.
     * When @p frameSize is set (e.g. NeuralDepth model input size), always requests that size.
     * When @p frameSize is unset and both stereo cameras already exist, uses ``Camera::getMaxWidth`` /
     * ``getMaxHeight`` (user ``sensorResolution`` or sensor maximum).
     * When Depth creates the cameras and @p frameSize is unset, uses full sensor resolution via
     * ``requestFullResolutionOutput``.
     */
    std::pair<Node::Output*, Node::Output*> ensureStereoOutputs(Pipeline& pipeline,
                                                                const StereoPair& pair,
                                                                std::optional<std::pair<uint32_t, uint32_t>> frameSize,
                                                                const std::optional<float>& fps);

    /** Stereo outputs for the device's first ``StereoPair`` (convenience over ``ensureStereoOutputs``). */
    std::pair<Node::Output*, Node::Output*> stereoCameraOutputs(Pipeline& pipeline,
                                                                const std::shared_ptr<Device>& device,
                                                                std::optional<std::pair<uint32_t, uint32_t>> frameSize);

    /** Map active backend depth/confidence outputs after backend nodes are created. */
    void bindBackendOutputs(Algorithm active);

    /** User-selected or ``AUTO`` algorithm; frozen after ``buildInternal()`` completes. */
    Algorithm algorithmOverride_;
    Algorithm resolvedAlgorithm_ = Algorithm::AUTO;
    /** True after lazy wiring in ``buildInternal()``; blocks further ``build()`` calls. */
    bool graphBuilt_{false};
    /** Optional FPS for stereo ``Camera`` outputs; set via ``build()`` before wiring. */
    std::optional<float> stereoOutputFps_{};
    std::optional<std::pair<uint32_t, uint32_t>> stereoSizeOverride_{};
    std::optional<DeviceModelZoo> neuralModelOverride_{};
    DeviceModelZoo resolvedNeuralModel_ = DeviceModelZoo::NEURAL_DEPTH_SMALL;
    StereoDepth::PresetMode resolvedStereoPreset_ = StereoDepth::PresetMode::DEFAULT;

    /** Resolved backend depth output (set in ``bindBackendOutputs``). */
    Node::Output* depthOut_{nullptr};
    /** Resolved backend confidence output, or null when the backend has none (GPUStereo). */
    Node::Output* confidenceOut_{nullptr};

    /** Populated when ``Algorithm::STEREO`` is active. */
    std::unique_ptr<::dai::Subnode<StereoDepth>> stereoBackend_;
    /** Populated when ``Algorithm::NEURAL`` is active. */
    std::unique_ptr<::dai::Subnode<NeuralDepth>> neuralBackend_;
    /** Populated when ``Algorithm::GPU_STEREO`` is active. */
    std::unique_ptr<::dai::Subnode<GPUStereo>> gpuStereoBackend_;
    /** Populated when ``Algorithm::NEURAL_ASSISTED_STEREO`` is active. */
    std::shared_ptr<NeuralAssistedStereo> nasBackend_;
    /** Populated when ``Algorithm::TOF`` is active. */
    std::shared_ptr<ToF> tofBackend_;
};

}  // namespace node
}  // namespace dai
