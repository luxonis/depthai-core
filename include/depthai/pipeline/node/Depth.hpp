#pragma once

#include <memory>
#include <optional>
#include <string>
#include <utility>
#include <variant>
#include <vector>

#include "depthai/beta/node/ToFStereoFusion.hpp"
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
 * @brief Depth node. Unified depth output from StereoDepth, NeuralDepth, NeuralAssistedStereo, ToF, or GPUStereo.
 *
 * With Algorithm::AUTO, the backend is chosen from device capabilities, target FPS, and stereo resolution.
 * On RVC4 this prefers NeuralDepth when available; on other platforms it uses ToF when a ToF sensor is
 * connected, otherwise StereoDepth.
 *
 * Use build() to pin algorithm, FPS, or resolution before the first output access or hasRectifiedOutputs() call.
 * Use setAlignTo() to align depth to another camera output.
 */
class Depth : public DeviceNodeGroup {
   public:
    /**
     * Backend selection for the Depth node.
     */
    enum class Algorithm : std::uint32_t {
        AUTO = 0,
        STEREO,
        NEURAL,
        NEURAL_ASSISTED_STEREO,
        TOF,
        GPU_STEREO,
        TOF_STEREO_FUSION,
    };

    /**
     * Algorithm-specific configuration (DeviceModelZoo for NEURAL, StereoDepth::PresetMode for STEREO,
     * or std::monostate for backends without a selectable profile).
     */
    using Config = std::variant<std::monostate, DeviceModelZoo, StereoDepth::PresetMode>;

    // Non-copyable / non-movable: owns pipeline subnodes and output pointers tied to this instance.
    Depth(const Depth&) = delete;
    Depth& operator=(const Depth&) = delete;
    Depth(Depth&&) = delete;
    Depth& operator=(Depth&&) = delete;

    ~Depth() override = default;

    Depth();

    /**
     * Create Depth node with Algorithm::AUTO.
     */
    [[nodiscard]] static std::shared_ptr<Depth> create() {
        return std::shared_ptr<Depth>(new Depth());
    }

    /**
     * Create Depth node with an optional device.
     * @param device Device to use; pipeline default is used when null
     */
    [[nodiscard]] static std::shared_ptr<Depth> create(const std::shared_ptr<Device>& device) {
        auto ptr = create();
        if(device != nullptr) {
            ptr->setDevice(device);
        }
        return ptr;
    }

    /**
     * Set stereo camera FPS for stereo-based backends.
     * Must be called before first output access or hasRectifiedOutputs() call.
     * @param fps Requested FPS; uses Camera default if not set
     */
    std::shared_ptr<Depth> build(std::optional<float> fps = std::nullopt);

    /**
     * Set algorithm and optional FPS before wiring.
     * @param algorithm Backend to use
     * @param fps Requested stereo camera FPS
     */
    std::shared_ptr<Depth> build(Algorithm algorithm, std::optional<float> fps = std::nullopt);

    /**
     * Set algorithm, optional FPS, and preferred depth output size before wiring.
     * @param algorithm Backend to use
     * @param fps Requested stereo camera FPS
     * @param size preferred depth output size. Used when no upstream Camera is present
     */
    std::shared_ptr<Depth> build(Algorithm algorithm, std::optional<float> fps, std::optional<std::pair<uint32_t, uint32_t>> size);

    /**
     * Set algorithm and config before wiring. algorithm must not be AUTO.
     * @param algorithm Backend to use
     * @param config Algorithm-specific configuration
     * @param fps Requested stereo camera FPS
     * @param size preferred depth output size. Used when no upstream Camera is present
     */
    std::shared_ptr<Depth> build(Algorithm algorithm,
                                 Config config,
                                 std::optional<float> fps = std::nullopt,
                                 std::optional<std::pair<uint32_t, uint32_t>> size = std::nullopt);

    /**
     * Output depth map from the active backend.
     */
    Node::Output& depth();

    /**
     * Output confidence map from the active backend.
     * When ToF is active, this forwards the actual ToF confidence output.
     */
    Node::Output& confidence();

    /**
     * Resolve AUTO and report whether the selected backend exposes a rectified stereo pair.
     * Returns false for TOF, without changing backend selection or creating stereo cameras.
     * Like depth(), this wires the graph and locks build()/setAlgorithm()/setConfig()/setAlignTo().
     * Requires the node to be attached to a pipeline with a device.
     */
    bool hasRectifiedOutputs();

    /**
     * Left rectified image used by the selected stereo-based backend.
     * Resolves AUTO and wires the graph on first access. Throws for TOF; use
     * hasRectifiedOutputs() when stereo images are optional. To require them,
     * select a stereo-based algorithm explicitly before accessing any output.
     *
     * These are the backend's rectified inputs, not aligned to setAlignTo().
     * Their size/format and timing follow the backend; depth postprocessing may
     * change depth resolution. For TOF_STEREO_FUSION these belong to the neural
     * stereo branch, while fused depth is in the ToF sensor's frame.
     */
    Node::Output& rectifiedLeft();

    /**
     * Right rectified image used by the selected stereo-based backend.
     * Has the same availability and geometry contract as rectifiedLeft().
     */
    Node::Output& rectifiedRight();

    /**
     * Align depth output to another image source.
     * Must be called before first output access or hasRectifiedOutputs() call.
     * Only depth() is aligned; confidence() stays in the backend frame.
     * @param alignTo Output to align depth to
     */
    std::shared_ptr<Depth> setAlignTo(Node::Output& alignTo);

    /**
     * Get the requested algorithm selection.
     */
    [[nodiscard]] Algorithm getRequestedAlgorithm() const {
        return algorithmOverride_;
    }

    /**
     * Get depth methods supported by a device and its connected sensors.
     * Can be called before creating a pipeline or Depth node.
     * Availability does not guarantee a particular FPS or resolution.
     * @param device Device to query; must not be null.
     * @return Supported concrete algorithms (AUTO is excluded); empty if none are supported.
     * @throws std::runtime_error If device is null.
     */
    [[nodiscard]] static std::vector<Algorithm> getSupportedAlgorithms(const std::shared_ptr<Device>& device);

    /**
     * Get the algorithm AUTO would select without FPS, size, config, or existing camera overrides.
     * Uses the device's native stereo resolution and a target of 30 FPS.
     * Can be called before creating a pipeline or Depth node.
     * @param device Device to query; must not be null.
     * @return The concrete default algorithm (never AUTO).
     * @throws std::runtime_error If device is null or lacks the sensors required for AUTO selection.
     */
    [[nodiscard]] static Algorithm getDefaultAlgorithm(const std::shared_ptr<Device>& device);

    /**
     * Set the requested algorithm before wiring.
     * @param algorithm Backend to use; AUTO re-enables auto-selection
     */
    std::shared_ptr<Depth> setAlgorithm(Algorithm algorithm);

    /**
     * Get the requested config override, if any.
     * @return Config override, or std::nullopt when config is auto-picked
     */
    [[nodiscard]] const std::optional<Config>& getRequestedConfig() const {
        return configOverride_;
    }

    /**
     * Pin the algorithm-specific config.
     * Only takes effect together with a concrete (non-AUTO) algorithm.
     * @param config Algorithm-specific configuration
     */
    std::shared_ptr<Depth> setConfig(Config config);

    /**
     * Get the algorithm actually wired (AUTO resolved).
     * Valid after first output access or hasRectifiedOutputs() call.
     */
    [[nodiscard]] Algorithm getResolvedAlgorithm() const {
        return resolved_.algorithm;
    }

    /**
     * Get the resolved algorithm-specific config.
     */
    [[nodiscard]] const Config& getResolvedConfig() const {
        return resolved_.config;
    }

    void buildInternal() override;

   private:
    friend struct DepthTestAccess;

    struct Selection {
        Algorithm algorithm;
        Config config;
    };

    struct StereoWiring {
        Node::Output* left{};
        Node::Output* right{};
        std::pair<uint32_t, uint32_t> resolution{};
        float maxCameraFps{0.f};
    };

    void requireNotBuilt(const char* method) const;

    static bool exceedsStereoDepthMaxResolution(uint32_t width, uint32_t height);

    static std::vector<Algorithm> getSupportedAlgorithmsWithModels(const std::shared_ptr<Device>& device, std::optional<std::vector<DeviceModelZoo>> models);

    // Keep selection independent of a live device and use the cached OS version to choose FPS limits.
    static Selection selectBackend(Platform platform,
                                   std::optional<std::pair<uint32_t, uint32_t>> resolution,
                                   float targetFps,
                                   const std::vector<Algorithm>& supportedAlgorithms,
                                   const std::vector<DeviceModelZoo>& modelFilter = {},
                                   bool requireFpsAndResolutionMatch = false,
                                   const std::optional<Version>& osVersion = std::nullopt);

    void resolveWiring(const std::shared_ptr<Device>& device, Pipeline& pipeline);

    StereoWiring ensureStereoOutputs(Pipeline& pipeline,
                                     const StereoPair& pair,
                                     std::optional<std::pair<uint32_t, uint32_t>> frameSize,
                                     const std::optional<float>& fps);

    void wireAlignment(Algorithm active, const std::shared_ptr<Device>& device);

    Algorithm algorithmOverride_{Algorithm::AUTO};
    Selection resolved_{Algorithm::AUTO, std::monostate{}};
    bool graphBuilt_{false};
    std::optional<float> stereoOutputFps_{};
    std::optional<std::pair<uint32_t, uint32_t>> sizeOverride_{};
    std::optional<Config> configOverride_{};
    Node::Output* alignToOutput_{nullptr};

    Node::Output* depthOut_{nullptr};
    Node::Output* confidenceOut_{nullptr};
    Node::Output* rectifiedLeftOut_{nullptr};
    Node::Output* rectifiedRightOut_{nullptr};

    std::unique_ptr<::dai::Subnode<StereoDepth>> stereoBackend_;
    std::unique_ptr<::dai::Subnode<NeuralDepth>> neuralBackend_;
    std::unique_ptr<::dai::Subnode<GPUStereo>> gpuStereoBackend_;
    std::shared_ptr<NeuralAssistedStereo> nasBackend_;
    std::shared_ptr<ToF> tofBackend_;
    std::shared_ptr<beta::node::ToFStereoFusion> tofStereoFusionBackend_;
    std::unique_ptr<::dai::Subnode<ImageAlign>> imageAlignBackend_;
};

}  // namespace node
}  // namespace dai
