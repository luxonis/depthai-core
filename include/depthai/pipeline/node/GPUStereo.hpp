#pragma once

#include <depthai/pipeline/DeviceNode.hpp>
#include <depthai/pipeline/Subnode.hpp>
#include <depthai/pipeline/datatype/GPUStereoConfig.hpp>
#include <depthai/pipeline/node/MessageDemux.hpp>
#include <depthai/pipeline/node/Rectification.hpp>
#include <depthai/pipeline/node/Sync.hpp>
#include <depthai/properties/GPUStereoProperties.hpp>

namespace dai {
namespace node {

/**
 * @brief GPU-accelerated stereo depth node for RVC4.
 *
 * Computes disparity and depth maps from a synchronized stereo camera pair using
 * OpenCL on the Adreno GPU. Supports both rectified and unrectified inputs
 * (controlled via @ref setRectification).
 */
class GPUStereo : public DeviceNodeCRTP<DeviceNode, GPUStereo, GPUStereoProperties> {
   protected:
    using DeviceNodeCRTP::DeviceNodeCRTP;
    Properties& getProperties() override;
    GPUStereo(std::unique_ptr<Properties> props);

   public:
    constexpr static const char* NAME = "GPUStereo";
    GPUStereo() = default;

    /**
     * @brief Build the node by linking left and right camera outputs.
     */
    std::shared_ptr<GPUStereo> build(Output& left, Output& right);

    /**
     * @brief Enable or disable built-in stereo rectification.
     *
     * When enabled, the node rectifies the input images internally using calibration data.
     * When disabled, inputs are expected to be already rectified.
     */
    GPUStereo& setRectification(bool enable);

    /**
     * @brief Set the confidence threshold for disparity filtering.
     *
     * Pixels with a matching cost above this threshold are invalidated.
     * @param threshold Value in range [0, 255]. 0 disables the filter.
     */
    GPUStereo& setConfidenceThreshold(int threshold);

    Subnode<Sync> sync{*this, "sync"};
    Subnode<MessageDemux> messageDemux{*this, "messageDemux"};
    Subnode<Rectification> rectification{*this, "rectification"};

#ifndef DEPTHAI_INTERNAL_DEVICE_BUILD_RVC4
    Input& left{sync->inputs["left"]};
    Input& right{sync->inputs["right"]};

    Output& rectifiedLeft{rectification->output1};
    Output& rectifiedRight{rectification->output2};
#endif

    Input leftInternal{*this, {"leftFrameInternal", DEFAULT_GROUP, false, 1, {{{DatatypeEnum::ImgFrame, false}}}}};
    Input rightInternal{*this, {"rightFrameInternal", DEFAULT_GROUP, false, 1, {{{DatatypeEnum::ImgFrame, false}}}}};

    Output disparity{*this, {"disparity", DEFAULT_GROUP, {{{DatatypeEnum::ImgFrame, false}}}}};
    Output depth{*this, {"depth", DEFAULT_GROUP, {{{DatatypeEnum::ImgFrame, false}}}}};

    void buildInternal() override;

    /**
     * Initial config to use for GPUStereo.
     *
     * Note: Only `confidenceThreshold` is supported/exposed for this node.
     */
    std::shared_ptr<GPUStereoConfig> initialConfig = std::make_shared<GPUStereoConfig>();
};

}  // namespace node
}  // namespace dai
