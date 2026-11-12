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

namespace dai::beta::node {

/**
 * @brief Experimental node that fuses aligned ToF and neural stereo-depth measurements.
 *
 * The node configures its internal ToF, rectification, synchronization, and fusion-network stages when built from a left and right camera.
 *
 * One network takes the rectified stereo pair and the ToF reprojected into the rectified-left view, and the device composes its
 * disparity with the ToF. The network ships with the RVC4 firmware; neuralNetwork->setModelPath() replaces it. The cameras are
 * scaled to the network input (1024x640) and rectified on the EVA warp with alpha scaling 0. Each stereo frame is fused with the
 * newest ToF frame (at most 100 ms older), so the ToF does not add to the latency. NeuralDepth is not used: build() removes it
 * from the pipeline.
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
     * @param fps Frame rate for the camera inputs and ToF subnode. Must be finite and positive. Defaults to 30 FPS.
     * @return This node.
     */
    std::shared_ptr<ToFStereoFusion> build(const std::shared_ptr<dai::node::Camera>& left, const std::shared_ptr<dai::node::Camera>& right, float fps = 30.0f);

    /** Initial confidence threshold and optional geometric-overlap crop configuration. */
    std::shared_ptr<ToFStereoFusionConfig> initialConfig;

   public:
    /** Internal ToF pipeline; its depth output is available for downstream use. */
    Subnode<dai::node::ToF> tof{*this, "tof"};
    /** Not used by the fusion: build() removes it from the pipeline, so its outputs carry no data. */
    Subnode<dai::node::NeuralDepth> neuralDepth{*this, "neuralDepth"};
    /** Internal fusion network. */
    Subnode<dai::node::NeuralNetwork> neuralNetwork{*this, "neuralNetwork"};

   private:
    Subnode<dai::node::Sync> sync{*this, "sync"};

   public:
    /** Fused depth output in millimeters, in the rectified-left view, with the rectified-left transformation. */
    Output depth{*this, {"depth", DEFAULT_GROUP, {{{DatatypeEnum::ImgFrame, false}}}}};
    /** Calibrated confidence of the fused depth (u8, higher is better, unthresholded), with the same transformation as depth. */
    Output confidence{*this, {"confidence", DEFAULT_GROUP, {{{DatatypeEnum::ImgFrame, false}}}}};

#ifndef DEPTHAI_INTERNAL_DEVICE_BUILD_RVC4
    /** Not used by the fusion (an input of the NeuralDepth subnode, which build() removes); link the cameras with build(). */
    Input& inputLeft{neuralDepth->sync->inputs["left"]};
    /** Not used by the fusion (an input of the NeuralDepth subnode, which build() removes); link the cameras with build(). */
    Input& inputRight{neuralDepth->sync->inputs["right"]};
#endif

   protected:
    Input syncedInputs{*this, {"syncedInputs", DEFAULT_GROUP, false, 4, {{{DatatypeEnum::MessageGroup, false}}}, DEFAULT_WAIT_FOR_MESSAGE}};
    Input nnDataInput{*this, {"nnDataInput", DEFAULT_GROUP, false, 2, {{{DatatypeEnum::NNData, false}}}, DEFAULT_WAIT_FOR_MESSAGE}};
    Output nnInput{*this, {"nnInput", DEFAULT_GROUP, {{{DatatypeEnum::NNData, false}}}}};

   private:
    Properties& getProperties() override;
    void buildInternal() override;

    std::unique_ptr<Subnode<dai::node::Sync>> pairSync;
    std::unique_ptr<Subnode<dai::node::MessageDemux>> pairDemux;
    std::unique_ptr<Subnode<dai::node::Rectification>> rectification;
    std::unique_ptr<Subnode<dai::node::Sync>> tofSync;
};

}  // namespace dai::beta::node
