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
 * @brief Depth node. Unified depth output from StereoDepth, NeuralDepth, NeuralAssistedStereo, ToF, or GPUStereo.
 *
 * With Algorithm::AUTO, the backend is chosen from device capabilities, target FPS, and stereo resolution.
 * On RVC4 this prefers NeuralDepth when available; on other platforms it uses ToF when a ToF sensor is
 * connected, otherwise StereoDepth.
 *
 * Use build() to pin algorithm or config, then requestOutput() to request a coupled depth/confidence output pair.
 * Use setAlignTo() to align depth to another camera output.
 */
class Depth : public DeviceNodeGroup {
   public:
    using DeviceNodeGroup::requestOutput;

    class RequestedOutput {
       public:
        class OutputHandle {
           public:
            Node::Output& get() const;
            Node::Output* operator->() const;
            operator Node::Output&() const;

           private:
            friend class RequestedOutput;
            using Resolver = Node::Output& (Depth::*)();
            OutputHandle(Depth& parent, Resolver resolver) : parent_(parent), resolver_(resolver) {}

            Depth& parent_;
            Resolver resolver_;
        };

        OutputHandle depth;
        OutputHandle confidence;

       private:
        friend class Depth;
        explicit RequestedOutput(Depth& parent) : depth(parent, &Depth::resolveDepthOutput), confidence(parent, &Depth::resolveConfidenceOutput) {}
    };

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
     * Build Depth with Algorithm::AUTO.
     */
    std::shared_ptr<Depth> build();

    /**
     * Set algorithm before wiring.
     * @param algorithm Backend to use
     */
    std::shared_ptr<Depth> build(Algorithm algorithm);

    /**
     * Request the default coupled depth/confidence output pair before wiring.
     * This uses the backend's default sizing behavior and optional requested FPS.
     * @param fps Optional requested stereo camera FPS for the output
     */
    RequestedOutput* requestOutput(std::optional<float> fps = std::nullopt);

    /**
     * Request a coupled depth/confidence output pair before wiring.
     * This drives AUTO backend selection and the auto-created stereo camera outputs when needed.
     * @param size Requested depth output size
     * @param fps Optional requested stereo camera FPS for the output
     */
    RequestedOutput* requestOutput(const std::pair<uint32_t, uint32_t>& size, std::optional<float> fps = std::nullopt);

    /**
     * Set algorithm and config before wiring. algorithm must not be AUTO.
     * @param algorithm Backend to use
     * @param config Algorithm-specific configuration
     */
    std::shared_ptr<Depth> build(Algorithm algorithm, Config config);

    /**
     * Align depth output to another image source.
     * Must be called before first requested output access.
     * Only the requested depth output is aligned; confidence stays in the backend frame.
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
     * Valid after first requested output access.
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

    std::vector<Algorithm> getSupportedAlgorithms(const std::shared_ptr<Device>& device, const std::vector<DeviceModelZoo>& supportedModels) const;

    static bool exceedsStereoDepthMaxResolution(uint32_t width, uint32_t height);

    static Selection selectBackend(std::optional<std::pair<uint32_t, uint32_t>> resolution,
                                   float targetFps,
                                   const std::vector<Algorithm>& supportedAlgorithms,
                                   const std::vector<DeviceModelZoo>& modelFilter = {},
                                   bool requireFpsAndResolutionMatch = false);

    void resolveWiring(const std::shared_ptr<Device>& device, Pipeline& pipeline);

    StereoWiring ensureStereoOutputs(Pipeline& pipeline,
                                     const StereoPair& pair,
                                     std::optional<std::pair<uint32_t, uint32_t>> frameSize,
                                     const std::optional<float>& fps);

    void wireAlignment(Algorithm active, const std::shared_ptr<Device>& device);
    Node::Output& resolveDepthOutput();
    Node::Output& resolveConfidenceOutput();

    Algorithm algorithmOverride_{Algorithm::AUTO};
    Selection resolved_{Algorithm::AUTO, std::monostate{}};
    bool graphBuilt_{false};
    std::optional<float> stereoOutputFps_{};
    std::optional<std::pair<uint32_t, uint32_t>> stereoSizeOverride_{};
    std::optional<Config> configOverride_{};
    Node::Output* alignToOutput_{nullptr};

    Node::Output* depthOut_{nullptr};
    Node::Output* confidenceOut_{nullptr};
    std::unique_ptr<RequestedOutput> requestedOutput_;

    std::unique_ptr<::dai::Subnode<StereoDepth>> stereoBackend_;
    std::unique_ptr<::dai::Subnode<NeuralDepth>> neuralBackend_;
    std::unique_ptr<::dai::Subnode<GPUStereo>> gpuStereoBackend_;
    std::shared_ptr<NeuralAssistedStereo> nasBackend_;
    std::shared_ptr<ToF> tofBackend_;
    std::unique_ptr<::dai::Subnode<ImageAlign>> imageAlignBackend_;
};

}  // namespace node
}  // namespace dai
