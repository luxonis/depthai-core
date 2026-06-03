#pragma once

#include <depthai/pipeline/DeviceNode.hpp>
#include <depthai/pipeline/Subnode.hpp>
#include <depthai/pipeline/datatype/GPUStereoConfig.hpp>
#include <depthai/pipeline/node/MessageDemux.hpp>
#include <depthai/pipeline/node/Rectification.hpp>
#include <depthai/pipeline/node/Sync.hpp>
#include <depthai/properties/GPUStereoProperties.hpp>
#include <memory>

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

    Subnode<Sync> sync{*this, "sync"};
    Subnode<MessageDemux> messageDemux{*this, "messageDemux"};
    std::unique_ptr<Subnode<Rectification>> rectification;

#ifndef DEPTHAI_INTERNAL_DEVICE_BUILD_RVC4
    Input& left{sync->inputs["left"]};
    Input& right{sync->inputs["right"]};
#endif

    Input leftInternal{*this, {"leftFrameInternal", DEFAULT_GROUP, false, 1, {{{DatatypeEnum::ImgFrame, false}}}}};
    Input rightInternal{*this, {"rightFrameInternal", DEFAULT_GROUP, false, 1, {{{DatatypeEnum::ImgFrame, false}}}}};

    /**
     * Outputs ImgFrame message that carries RAW16 encoded disparity data.
     */
    Output disparity{*this, {"disparity", DEFAULT_GROUP, {{{DatatypeEnum::ImgFrame, false}}}}};

    /**
     * Outputs ImgFrame message that carries RAW16 encoded (0..65535) depth data in depth units (millimeter by default).
     *
     * Non-determined / invalid depth values are set to 0
     */
    Output depth{*this, {"depth", DEFAULT_GROUP, {{{DatatypeEnum::ImgFrame, false}}}}};

    /**
     * Outputs ImgFrame message that carries RAW8 confidence map.
     * Lower values mean lower confidence of the calculated disparity value.
     * Note: postprocessing steps like LR-check/median filter are not applied to confidence map.
     */
    Output confidenceMap{*this, {"confidenceMap", DEFAULT_GROUP, {{{DatatypeEnum::ImgFrame, false}}}}};

    void buildInternal() override;

    /**
     * Initial config to use for GPUStereo.
     *
     * Use this to configure startup parameters before the pipeline starts.
     * Note: Only `confidenceThreshold` is supported/exposed for this node.
     */
    std::shared_ptr<GPUStereoConfig> initialConfig = std::make_shared<GPUStereoConfig>();

   private:
    bool rectificationEnabled = true;
};

}  // namespace node
}  // namespace dai
