#pragma once

#include <depthai/beta/properties/ToFStereoFusionProperties.hpp>
#include <depthai/pipeline/DeviceNodeGroup.hpp>
#include <depthai/pipeline/Subnode.hpp>
#include <depthai/pipeline/datatype/ImgFrame.hpp>
#include <depthai/pipeline/node/Camera.hpp>
#include <depthai/pipeline/node/MessageDemux.hpp>
#include <depthai/pipeline/node/NeuralDepth.hpp>
#include <depthai/pipeline/node/NeuralNetwork.hpp>
#include <depthai/pipeline/node/Rectification.hpp>
#include <depthai/pipeline/node/Sync.hpp>
#include <depthai/pipeline/node/ToF.hpp>
#include <memory>
#include <utility>

namespace dai::beta::node {

/**
 * @brief Experimental node that fuses aligned ToF and neural stereo-depth measurements.
 *
 * The node configures its internal ToF, neural-depth, alignment, synchronization, and fusion-network stages when built from a left and right camera.
 *
 * Two fusion models are available (setModel()):
 * - ALPHA_BLEND (default): NeuralDepth is aligned to the ToF and a small network blends the two per pixel. Output in the ToF view.
 * - STEREO_NETWORK: the rectified stereo pair and ToF planes (ToF reprojected into the rectified-left view) go into one network,
 *   whose disparity is composed with the ToF on the device. Output in the rectified-left view. The network is not in the model zoo;
 *   set it with neuralNetwork->setModelPath() before starting the pipeline. The stereo pair is rectified by a Rectification
 *   subnode on the EVA warp (Backend::HW) with alpha scaling 0, the geometry the networks are trained on.
 *
 * @note This node is supported on RVC4 devices only. Creating it for an RVC2 device throws an exception.
 */
class ToFStereoFusion : public DeviceNodeCRTP<DeviceNode, ToFStereoFusion, ToFStereoFusionProperties> {
   public:
    using Input = Node::Input;
    using Output = Node::Output;

    /** Node type name used during pipeline serialization. */
    constexpr static const char* NAME = "ToFStereoFusion";

    /** Creates a ToFStereoFusion node for the supplied device. */
    explicit ToFStereoFusion(const std::shared_ptr<Device>& device);
    /** Creates a ToFStereoFusion node from serialized properties. */
    explicit ToFStereoFusion(std::unique_ptr<Properties> props)
        : DeviceNodeCRTP(std::move(props)), initialConfig(std::make_shared<ToFStereoFusionConfig>(properties.initialConfig)) {}
    ~ToFStereoFusion() override;

    /**
     * Creates and initializes a ToFStereoFusion node for the supplied device.
     *
     * @param device Device that owns the node.
     * @return Initialized ToFStereoFusion node.
     */
    [[nodiscard]] static std::shared_ptr<ToFStereoFusion> create(const std::shared_ptr<Device>& device) {
        auto node = std::make_shared<ToFStereoFusion>(device);
        node->buildInternal();
        return node;
    }

    /**
     * Configures fusion from a synchronized left and right camera pair.
     *
     * @note This node is supported on RVC4 devices only.
     * @param left Left camera node.
     * @param right Right camera node.
     * @param fps Frame rate for the neural-depth camera inputs and ToF subnode. Must be finite and positive. Defaults to 30 FPS.
     * @return This node.
     */
    std::shared_ptr<ToFStereoFusion> build(const std::shared_ptr<dai::node::Camera>& left, const std::shared_ptr<dai::node::Camera>& right, float fps = 30.0f);

    /**
     * Select the fusion model. Call before build().
     *
     * @param model ALPHA_BLEND (default) or STEREO_NETWORK.
     * @return This node.
     */
    ToFStereoFusion& setModel(Properties::Model model);

    /** Get the fusion model. */
    Properties::Model getModel() const;

    /**
     * Set where the STEREO_NETWORK pre- and post-processing runs. AUTO (default) uses the GPU when available.
     *
     * @param backend AUTO, CPU or GPU. AUTO falls back to the CPU with a warning when OpenCL is not available; GPU does not
     *        fall back: the node logs an error and produces no output. The device logs the backend in use at info level.
     * @return This node.
     */
    ToFStereoFusion& setBackend(Properties::Backend backend);

    /**
     * Set the scale of the ToF confidence for the STEREO_NETWORK model.
     *
     * With FUSION (default), build() puts the ToF into ToFConfig::DepthMode::FUSION, whose confidence is already on the fusion
     * scale, and links its phase output. With VENDOR the ToF keeps its standard depth mode and the confidence is mapped to the
     * fusion scale; use it for networks trained on the standard ToF output. Call before build().
     * When the ToF cannot run the fusion depth mode (no GPU) or is switched to STANDARD at runtime, it sends empty phase frames
     * with vendor-scale confidence; the node keeps producing depth from those frames (phases derived from the ToF depth, the
     * confidence mapped as with VENDOR, lower quality) and logs a warning.
     *
     * @param scale FUSION or VENDOR.
     * @return This node.
     */
    ToFStereoFusion& setToFConfidenceScale(Properties::ToFConfidenceScale scale);

    /**
     * Set the size of the rectified stereo pair for the STEREO_NETWORK model. Call before build().
     *
     * The size must match the network input: at least 512x256, width a multiple of 128, height a multiple of 32.
     * The cameras are scaled, not cropped, so the field of view does not change. Defaults to 1280x800.
     *
     * @param width Width in pixels.
     * @param height Height in pixels.
     * @return This node.
     */
    ToFStereoFusion& setNetworkInputSize(uint32_t width, uint32_t height);

    /** Initial confidence threshold and optional geometric-overlap crop configuration. */
    std::shared_ptr<ToFStereoFusionConfig> initialConfig;

   public:
    /** Internal ToF pipeline; its depth output is available for downstream use. */
    Subnode<dai::node::ToF> tof{*this, "tof"};
    /** Internal neural stereo-depth pipeline; its depth output is available for downstream use. */
    Subnode<dai::node::NeuralDepth> neuralDepth{*this, "neuralDepth"};
    /** Internal alpha fusion network. */
    Subnode<dai::node::NeuralNetwork> neuralNetwork{*this, "neuralNetwork"};

   private:
    Subnode<dai::node::Sync> sync{*this, "sync"};

   public:
    /**
     * Fused depth output in millimeters. ALPHA_BLEND: aligned to the ToF sensor; optional cropping updates intrinsics and preserves the
     * ToF camera pose. STEREO_NETWORK: in the rectified-left view, with the rectified-left transformation.
     */
    Output depth{*this, {"depth", DEFAULT_GROUP, {{{DatatypeEnum::ImgFrame, false}}}}};
    /** Confidence output produced by the ToF-neural fusion network, with the same optional crop and transformation as depth. */
    Output confidence{*this, {"confidence", DEFAULT_GROUP, {{{DatatypeEnum::ImgFrame, false}}}}};

#ifndef DEPTHAI_INTERNAL_DEVICE_BUILD_RVC4
    /** Left camera input used when the node runs outside the RVC4 device build. */
    Input& inputLeft{neuralDepth->sync->inputs["left"]};
    /** Right camera input used when the node runs outside the RVC4 device build. */
    Input& inputRight{neuralDepth->sync->inputs["right"]};
#endif

   protected:
    Input syncedInputs{*this, {"syncedInputs", DEFAULT_GROUP, false, 4, {{{DatatypeEnum::MessageGroup, false}}}, DEFAULT_WAIT_FOR_MESSAGE}};
    Input nnDataInput{*this, {"nnDataInput", DEFAULT_GROUP, false, 2, {{{DatatypeEnum::NNData, false}}}, DEFAULT_WAIT_FOR_MESSAGE}};
    Output nnInput{*this, {"nnInput", DEFAULT_GROUP, {{{DatatypeEnum::NNData, false}}}}};

   private:
    Properties& getProperties() override;
    void buildInternal() override;
    void buildStereoNetwork(const std::shared_ptr<dai::node::Camera>& left, const std::shared_ptr<dai::node::Camera>& right, float fps);

    std::pair<uint32_t, uint32_t> networkInputSize{1280, 800};
    std::unique_ptr<Subnode<dai::node::Sync>> pairSync;
    std::unique_ptr<Subnode<dai::node::MessageDemux>> pairDemux;
    std::unique_ptr<Subnode<dai::node::Rectification>> rectification;
};

}  // namespace dai::beta::node
