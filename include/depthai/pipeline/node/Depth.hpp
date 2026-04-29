#pragma once

#include <memory>
#include <optional>

#include "depthai/common/CameraBoardSocket.hpp"
#include "depthai/common/DeviceModelZoo.hpp"
#include "depthai/common/StereoPair.hpp"
#include "depthai/device/Device.hpp"
#include "depthai/pipeline/DeviceNodeGroup.hpp"
#include "depthai/pipeline/Subnode.hpp"
#include "depthai/pipeline/datatype/ImageFiltersConfig.hpp"
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
 * ``Algorithm::AUTO`` uses NeuralDepth on RVC4 and StereoDepth elsewhere. Pass a non-``AUTO`` algorithm to ``create`` or
 * call ``setAlgorithm`` before the first ``depth()`` / ``confidence()`` access. ``confidence()`` maps to backend confidence when present; for GPUStereo it uses
 * ``disparity``; for ToF it uses ``amplitude``.
 *
 * Algorithm availability: ``TOF`` requires a connected ToF sensor (see ``Device::getConnectedCameraFeatures()``).
 * ``NEURAL_ASSISTED_STEREO`` is RVC4-only. ``GPU_STEREO`` is RVC4-only, requires a Kompute-enabled build, and excludes Lite-class SKUs
 * (heuristic on product name); extend checks as hardware matrix evolves.
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

    /** Create Depth; optional \p algorithm fixes the backend (default AUTO = platform-based). */
    [[nodiscard]] static std::shared_ptr<Depth> create(const std::shared_ptr<Device>& device,
                                                     Algorithm algorithm = Algorithm::AUTO) {
        return std::shared_ptr<Depth>(new Depth(device, algorithm));
    }

    std::shared_ptr<Depth> build(DeviceModelZoo neuralModel = DeviceModelZoo::NEURAL_DEPTH_SMALL) {
        neuralModel_ = neuralModel;
        return std::static_pointer_cast<Depth>(shared_from_this());
    }

    /** Override backend; validated against device when a Device is already set (non-AUTO). */
    std::shared_ptr<Depth> setAlgorithm(Algorithm algorithm) {
        algorithmOverride_ = algorithm;
        return std::static_pointer_cast<Depth>(shared_from_this());
    }

    /** Current algorithm selection (including AUTO). */
    [[nodiscard]] Algorithm getAlgorithm() const {
        return algorithmOverride_;
    }

    std::shared_ptr<Depth> setNeuralAssistedStereoModel(DeviceModelZoo model) {
        nasNeuralModel_ = model;
        return std::static_pointer_cast<Depth>(shared_from_this());
    }

    std::shared_ptr<Depth> setNeuralAssistedStereoRectify(bool rectify) {
        nasRectify_ = rectify;
        return std::static_pointer_cast<Depth>(shared_from_this());
    }

    std::shared_ptr<Depth> setTofOptions(CameraBoardSocket boardSocket = CameraBoardSocket::AUTO,
                                         ImageFiltersPresetMode presetMode = ImageFiltersPresetMode::TOF_MID_RANGE,
                                         std::optional<float> fps = std::nullopt) {
        tofSocket_ = boardSocket;
        tofPreset_ = presetMode;
        tofFps_ = std::move(fps);
        return std::static_pointer_cast<Depth>(shared_from_this());
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

    NeuralAssistedStereo* getNeuralAssistedStereo() const {
        return nasBackend_.get();
    }

    ToF* getToF() const {
        return tofBackend_.get();
    }

    GPUStereo* getGPUStereo() const {
        return gpuStereoBackend_ ? &**gpuStereoBackend_ : nullptr;
    }

   private:
    /** \p algorithm initialises override before first depth()/confidence() (same semantics as setAlgorithm). */
    explicit Depth(const std::shared_ptr<Device>& device, Algorithm algorithm = Algorithm::AUTO);

    Algorithm resolveAlgorithm(const std::shared_ptr<Device>& device) const;
    void validateAlgorithm(const std::shared_ptr<Device>& device, Algorithm active) const;

    std::pair<Node::Output*, Node::Output*> ensureStereoCameraOutputs(Pipeline& pipeline,
                                                                      const StereoPair& pair,
                                                                      std::pair<uint32_t, uint32_t> frameSize,
                                                                      float monoFps);

    std::pair<Node::Output*, Node::Output*> ensureStereoFullResolutionOutputs(Pipeline& pipeline,
                                                                              const StereoPair& pair,
                                                                              float monoFps);

    Algorithm algorithmOverride_;
    DeviceModelZoo neuralModel_{DeviceModelZoo::NEURAL_DEPTH_SMALL};
    DeviceModelZoo nasNeuralModel_{DeviceModelZoo::NEURAL_DEPTH_NANO};
    bool nasRectify_{true};
    CameraBoardSocket tofSocket_{CameraBoardSocket::AUTO};
    ImageFiltersPresetMode tofPreset_{ImageFiltersPresetMode::TOF_MID_RANGE};
    std::optional<float> tofFps_{};

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
