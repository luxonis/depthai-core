#pragma once

#include <depthai/pipeline/DeviceNode.hpp>
#include <depthai/pipeline/Subnode.hpp>
#include <depthai/pipeline/datatype/StereoDepthConfig.hpp>
#include <depthai/pipeline/datatype/VppConfig.hpp>
#include <depthai/pipeline/node/Rectification.hpp>
#include <depthai/pipeline/node/NeuralDepth.hpp>
#include <depthai/pipeline/node/Vpp.hpp>
#include <depthai/pipeline/node/StereoDepth.hpp>
#include <depthai/properties/NeuralAssistedStereoProperties.hpp>

namespace dai {
namespace node {

/**
 * @brief NeuralAssistedStereo node. Combines Neural Depth with VPP and traditional Stereo Depth.
 * 
 * This composite node internally creates and connects:
 * - Rectification node (full resolution)
 * - NeuralDepth node (low resolution depth estimation)
 * - VPP node (applies virtual projection pattern)
 * - StereoDepth node (final depth computation on VPP-enhanced images)
 * 
 * Pipeline structure:
 *   Left/Right Cameras → Rectification → [Full res to VPP]
 *                                      ↓
 *                             NeuralDepth (low res) → [disparity + confidence to VPP]
 *                                      ↓
 *                                    VPP (combines neural depth with full res images)
 *                                      ↓
 *                                 StereoDepth → Final Depth Output
 */
class NeuralAssistedStereo : public DeviceNodeCRTP<DeviceNode, NeuralAssistedStereo, NeuralAssistedStereoProperties> {
   public:
    constexpr static const char* NAME = "NeuralAssistedStereo";
    
    NeuralAssistedStereo() = default;
    NeuralAssistedStereo(std::unique_ptr<Properties> props); 
    


   protected:
    Properties& getProperties() override;
    using DeviceNodeCRTP::DeviceNodeCRTP;

    template <typename T>
    friend class Subnode;

   public:
    /**
     * Build the composite node by connecting left and right camera outputs
     * @param left Left camera output
     * @param right Right camera output
     * @param neuralModel Neural depth model to use
     * @return Shared pointer to this node
     */
    std::shared_ptr<NeuralAssistedStereo> build(Output& left, Output& right, DeviceModelZoo neuralModel = DeviceModelZoo::NEURAL_DEPTH_NANO);

    /**
     * Subnodes that compose this pipeline
     */
    Subnode<Rectification> rectification{*this, "rectification"};
    Subnode<NeuralDepth> neuralDepth{*this, "neuralDepth"};
    Subnode<Vpp> vpp{*this, "vpp"};
    Subnode<StereoDepth> stereoDepth{*this, "stereoDepth"};

    /**
     * Configure the VPP (Virtual Projection Pattern) parameters
     */
    std::shared_ptr<VppConfig> vppConfig = std::make_shared<VppConfig>();

    /**
     * Configure the StereoDepth parameters
     */
    std::shared_ptr<StereoDepthConfig> stereoConfig = std::make_shared<StereoDepthConfig>();

    /**
     * Configure the NeuralDepth parameters
     */
    std::shared_ptr<NeuralDepthConfig> neuralConfig = std::make_shared<NeuralDepthConfig>();

#ifndef DEPTHAI_INTERNAL_DEVICE_BUILD_RVC4
    /**
     * Input for left ImgFrame
     */
    Input& left{rectification->input1};

    /**
     * Input for right ImgFrame
     */
    Input& right{rectification->input2};

    /**
     * Output rectified left (full resolution)
     */
    Output& rectifiedLeft{rectification->output1};

    /**
     * Output rectified right (full resolution)
     */
    Output& rectifiedRight{rectification->output2};

    /**
     * Output VPP-enhanced left
     */
    Output& vppLeft{vpp->leftOut};

    /**
     * Output VPP-enhanced right
     */
    Output& vppRight{vpp->rightOut};

    /**
     * Neural depth disparity output
     */
    Output& neuralDisparity{neuralDepth->disparity};

    /**
     * Neural depth confidence output
     */
    Output& neuralConfidence{neuralDepth->confidence};
#endif

    /**
     * VPP configuration input
     */
    Input& inputVppConfig{vpp->inputConfig};
    
    /**
     * StereoDepth configuration input
     */
    Input& inputStereoConfig{stereoDepth->inputConfig};
    
    /**
     * NeuralDepth configuration input
     */
    Input& inputNeuralConfig{neuralDepth->inputConfig};

    /**
     * Final depth output from StereoDepth
     */
    Output& depth{stereoDepth->depth};

    /**
     * Disparity output from StereoDepth
     */
    Output& disparity{stereoDepth->disparity};

 
    void buildInternal() override;
};

}  // namespace node
}  // namespace dai
