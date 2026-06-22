#pragma once

#include <memory>
#include <optional>
#include <utility>
#include <variant>
#include <vector>

#include "depthai/common/DeviceModelZoo.hpp"
#include "depthai/common/StereoPair.hpp"
#include "depthai/device/Device.hpp"
#include "depthai/pipeline/DeviceNodeGroup.hpp"
#include "depthai/pipeline/Subnode.hpp"
#include "depthai/pipeline/node/Camera.hpp"
#include "depthai/pipeline/node/GPUStereo.hpp"
#include "depthai/pipeline/node/ImageAlign.hpp"
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
 * On RVC4 ``Algorithm::AUTO`` is resolved from target FPS and stereo resolution. FPS is taken
 * from ``build(fps)``, upstream stereo cameras, or defaults to 30. Resolution comes from
 * ``build(..., stereoSize)``, upstream cameras, or device camera features.
 *
 * Selection keeps the requested FPS first; if no backend can serve the resolution at that FPS,
 * it picks the highest-FPS backend profile that covers the resolution. Each backend uses a
 * ``Config`` variant (model, preset, or none).
 * On other platforms, AUTO uses ToF when a ToF sensor is connected, otherwise StereoDepth.
 *
 * ``confidence()`` maps to backend confidence when present: StereoDepth/NAS confidence,
 * NeuralDepth confidence, GPUStereo confidence map, or ToF amplitude.
 *
 * ``setAlignTo()`` optionally aligns the depth output to another image source. On RVC4 an
 * ``ImageAlign`` node is used for every backend; on RVC2 StereoDepth aligns natively via
 * ``inputAlignTo`` while other backends (e.g. ToF) use an on-host ``ImageAlign`` node.
 *
 * Algorithm availability: ``TOF`` requires a connected ToF sensor (see ``Device::getConnectedCameraFeatures()``).
 * ``NEURAL_ASSISTED_STEREO`` is RVC4-only. ``GPU_STEREO`` is RVC4-only, requires a stereo pair and is gated by
 * ``Device::isGpuStereoSupported()``.
 *
 * See \ref depth_node for a dedicated overview of purpose, behavior, features, and constraints.
 */
class Depth : public DeviceNodeGroup {
   public:
    /** Backend selection for the composite Depth node. */
    enum class Algorithm : std::uint32_t {
        AUTO = 0,                ///< Resolved at wiring via device capabilities and stereo size / FPS (RVC4).
        STEREO,                  ///< Classic StereoDepth disparity/depth pipeline.
        NEURAL,                  ///< RVC4 NeuralDepth; model auto-selected on RVC4 unless overridden.
        NEURAL_ASSISTED_STEREO,  ///< RVC4 NeuralAssistedStereo (stereo + neural assist).
        TOF,                     ///< Time-of-flight depth when a ToF sensor is connected.
        GPU_STEREO,              ///< RVC4 GPUStereo (GPU-accelerated disparity/depth with confidence map).
    };

    /**
     * Algorithm-specific configuration: a ``DeviceModelZoo`` model for ``NEURAL``,
     * a ``StereoDepth::PresetMode`` for ``STEREO``, or ``std::monostate`` for backends with
     * no selectable profile (``NEURAL_ASSISTED_STEREO``, ``GPU_STEREO``, ``TOF``).
     */
    using Config = std::variant<std::monostate, DeviceModelZoo, StereoDepth::PresetMode>;

    // Non-copyable / non-movable: owns pipeline subnodes and output pointers tied to this instance.
    Depth(const Depth&) = delete;
    Depth& operator=(const Depth&) = delete;
    Depth(Depth&&) = delete;
    Depth& operator=(Depth&&) = delete;

    ~Depth() override = default;

    /** Default ``Algorithm::AUTO`` (resolved when the graph is wired). */
    Depth();

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
    [[nodiscard]] static std::shared_ptr<Depth> create(const std::shared_ptr<Device>& device) {
        auto ptr = create();
        if(device != nullptr) {
            ptr->setDevice(device);
        }
        return ptr;
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
    std::shared_ptr<Depth> build(Algorithm algorithm, std::optional<float> fps, std::optional<std::pair<uint32_t, uint32_t>> stereoSize);

    /**
     * Set an explicit ``algorithm`` together with its ``config`` before first wiring.
     * Both must be supplied; the pair is used exactly as given (no auto algorithm/profile picking).
     * ``algorithm`` must not be ``AUTO``, ``config`` must match the algorithm's ``Config`` type, and
     * the algorithm must be available on the device or wiring throws with a descriptive message.
     */
    std::shared_ptr<Depth> build(Algorithm algorithm,
                                 Config config,
                                 std::optional<float> fps = std::nullopt,
                                 std::optional<std::pair<uint32_t, uint32_t>> stereoSize = std::nullopt);

    /**
     * Optionally align the depth output to another image source (e.g. a color ``Camera`` output).
     * Must be called before the first ``depth()`` / ``confidence()`` access (lazy wiring).
     *
     * The alignment is wired according to the platform and the resolved backend:
     * - RVC4: an ``ImageAlign`` node (on-device) aligns the backend depth for every backend.
     * - RVC2 + StereoDepth: the native ``StereoDepth::inputAlignTo`` performs the alignment on-device.
     * - RVC2 + any other backend (e.g. ToF): an ``ImageAlign`` node running on host
     *   (``setRunOnHost(true)``) aligns the backend depth.
     *
     * Only ``depth()`` is aligned; ``confidence()`` keeps the backend's native frame of reference.
     */
    std::shared_ptr<Depth> setAlignTo(Node::Output& alignTo);

    /** Requested algorithm selection (including ``AUTO``). */
    [[nodiscard]] Algorithm getRequestedAlgorithm() const {
        return algorithmOverride_;
    }

    /** Set the requested algorithm before wiring (``AUTO`` re-enables auto-selection). */
    std::shared_ptr<Depth> setAlgorithm(Algorithm algorithm);

    /**
     * Requested config override, if one was supplied via ``setConfig()`` / ``build(algorithm, config)``.
     * ``std::nullopt`` means the config is auto-picked for the algorithm.
     */
    [[nodiscard]] const std::optional<Config>& getRequestedConfig() const {
        return configOverride_;
    }

    /**
     * Pin the algorithm-specific config (``DeviceModelZoo`` for ``NEURAL``, ``StereoDepth::PresetMode``
     * for ``STEREO``, ``std::monostate`` for ``NEURAL_ASSISTED_STEREO`` / ``GPU_STEREO`` / ``TOF``).
     * Only takes effect together with a concrete (non-``AUTO``) algorithm.
     */
    std::shared_ptr<Depth> setConfig(Config config);

    /** Algorithm actually wired (``AUTO`` resolved). Valid after first ``depth()`` access. */
    [[nodiscard]] Algorithm getResolvedAlgorithm() const {
        return resolved_.algorithm;
    }

    /** Resolved algorithm-specific config (model, preset, or ``std::monostate``): see ``Config``. */
    [[nodiscard]] const Config& getResolvedConfig() const {
        return resolved_.config;
    }

    /** Lazily wires the selected backend subgraph when the node is attached to a pipeline. */
    void buildInternal() override;

    /** Depth map from the active backend; triggers ``buildInternal()`` on first access. */
    Node::Output& depth();
    /**
     * Confidence map from the active backend: StereoDepth/NAS confidence, NeuralDepth confidence,
     * GPUStereo confidence map, or ToF amplitude. Triggers ``buildInternal()`` on first access.
     */
    Node::Output& confidence();

   private:
    friend struct DepthTestAccess;

    /** Pair returned by the auto picker: the wired algorithm and its profile. */
    struct Selection {
        Algorithm algorithm;
        Config config;
    };

    /** Common pre-wiring guard for build/setter APIs. */
    void requireNotBuilt(const char* method) const;

    /**
     * Returns the algorithms usable on @p device. ``NEURAL`` is included only when the device
     * reports NeuralDepth support *and* @p supportedModels contains at least one model-zoo depth
     * model (so AUTO never selects NeuralDepth when no model is actually available).
     */
    std::vector<Algorithm> getSupportedAlgorithms(const std::shared_ptr<Device>& device, const std::vector<DeviceModelZoo>& supportedModels) const;

    /** True when @p width or @p height exceeds the StereoDepth maximum. */
    static bool exceedsStereoDepthMaxResolution(uint32_t width, uint32_t height);

    /**
     * RVC4 auto-selection: keep @p targetFps, try NeuralDepth (best model meeting FPS and optional
     * @p resolution), then NeuralAssistedStereo, StereoDepth, GPUStereo — first match wins.
     *
     * When @p requireFpsAndResolutionMatch is set (the user explicitly pinned both FPS and
     * resolution), the function throws instead of falling back if no backend can serve both.
     */
    static Selection selectBackend(std::optional<std::pair<uint32_t, uint32_t>> resolution,
                                   float targetFps,
                                   const std::vector<Algorithm>& supportedAlgorithms,
                                   const std::vector<DeviceModelZoo>& modelFilter = {},
                                   bool requireFpsAndResolutionMatch = false);

    /** Resolve algorithm + config, then wire stereo inputs from pipeline cameras / build() overrides. */
    void resolveWiring(const std::shared_ptr<Device>& device, Pipeline& pipeline);

    /** Left/right camera outputs plus the resolution and FPS actually wired for the stereo pair. */
    struct StereoWiring {
        Node::Output* left{};
        Node::Output* right{};
        std::pair<uint32_t, uint32_t> resolution{};
        float maxCameraFps{0.f};
    };

    /**
     * Ensures stereo cameras exist and returns left/right outputs.
     * When @p frameSize is set (e.g. NeuralDepth model input size), always requests that size.
     * When @p frameSize is unset and both stereo cameras already exist, uses ``Camera::getMaxWidth`` /
     * ``getMaxHeight`` (user ``sensorResolution`` or sensor maximum).
     * When Depth creates the cameras and @p frameSize is unset, uses full sensor resolution via
     * ``requestFullResolutionOutput``.
     */
    StereoWiring ensureStereoOutputs(Pipeline& pipeline,
                                     const StereoPair& pair,
                                     std::optional<std::pair<uint32_t, uint32_t>> frameSize,
                                     const std::optional<float>& fps);

    /**
     * Wire the optional ``alignTo`` source to the active backend (no-op when ``alignTo`` is unset).
     * RVC4 always inserts an on-device ``ImageAlign`` node; RVC2 uses ``StereoDepth::inputAlignTo``
     * for the StereoDepth backend and an on-host ``ImageAlign`` node for all other backends.
     * Updates ``depthOut_`` to the aligned output when an ``ImageAlign`` node is used.
     */
    void wireAlignment(Algorithm active, const std::shared_ptr<Device>& device);

    /** User-selected or ``AUTO`` algorithm; frozen after ``buildInternal()`` completes. */
    Algorithm algorithmOverride_{Algorithm::AUTO};
    /** Resolved algorithm + config; populated by ``resolveWiring()``. */
    Selection resolved_{Algorithm::AUTO, std::monostate{}};
    /** True after lazy wiring in ``buildInternal()``; blocks further ``build()`` calls. */
    bool graphBuilt_{false};
    /** Optional FPS for stereo ``Camera`` outputs; set via ``build()`` before wiring. */
    std::optional<float> stereoOutputFps_{};
    /** Optional user resolution (e.g. when no upstream ``Camera`` exists). */
    std::optional<std::pair<uint32_t, uint32_t>> stereoSizeOverride_{};
    /** Optional exact config supplied with a concrete algorithm; skips auto profile picking. */
    std::optional<Config> configOverride_{};
    /** Optional source output the depth is aligned to; set via ``setAlignTo()`` before wiring. */
    Node::Output* alignToOutput_{nullptr};

    /** Resolved backend depth output (set while wiring the active backend). */
    Node::Output* depthOut_{nullptr};
    /** Resolved backend confidence output (set while wiring the active backend). */
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
    /** Populated when ``alignTo`` is set and the alignment is performed by an ``ImageAlign`` node. */
    std::unique_ptr<::dai::Subnode<ImageAlign>> imageAlignBackend_;
};

}  // namespace node
}  // namespace dai
