#pragma once

#include <depthai/beta/properties/ToFStereoFusionProperties.hpp>
#include <depthai/pipeline/DeviceNodeGroup.hpp>
#include <depthai/pipeline/Subnode.hpp>
#include <depthai/pipeline/datatype/ImgFrame.hpp>
#include <depthai/pipeline/node/Camera.hpp>
#include <depthai/pipeline/node/ImageAlign.hpp>
#include <depthai/pipeline/node/NeuralDepth.hpp>
#include <depthai/pipeline/node/NeuralNetwork.hpp>
#include <depthai/pipeline/node/Sync.hpp>
#include <depthai/pipeline/node/ToF.hpp>

namespace dai::beta::node {

/**
 * @brief Experimental node that fuses aligned ToF and neural stereo-depth measurements.
 *
 * The node configures its internal ToF, neural-depth, alignment, synchronization, and fusion-network stages when built from a left and right camera.
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
     * @return This node.
     */
    std::shared_ptr<ToFStereoFusion> build(const std::shared_ptr<dai::node::Camera>& left, const std::shared_ptr<dai::node::Camera>& right);

   private:
    Subnode<dai::node::ToF> tof{*this, "tof"};
    Subnode<dai::node::NeuralDepth> neuralDepth{*this, "neuralDepth"};
    Subnode<dai::node::NeuralNetwork> neuralNetwork{*this, "neuralNetwork"};
    Subnode<dai::node::ImageAlign> neuralDepthAlign{*this, "neuralDepthAlign"};
    Subnode<dai::node::Sync> sync{*this, "sync"};

   public:
    /** Fused depth output, aligned to the ToF sensor. */
    Output depth{*this, {"depth", DEFAULT_GROUP, {{{DatatypeEnum::ImgFrame, false}}}}};
    /** Confidence output produced by the ToF-neural fusion network. */
    Output neuralConfidence{*this, {"neuralConfidence", DEFAULT_GROUP, {{{DatatypeEnum::ImgFrame, false}}}}};

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
};

}  // namespace dai::beta::node
