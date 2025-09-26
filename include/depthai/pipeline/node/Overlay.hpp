#pragma once

#include <depthai/pipeline/DeviceNode.hpp>

// shared
#include <depthai/properties/OverlayProperties.hpp>

#include "depthai/pipeline/datatype/ImgFrame.hpp"

namespace dai {
namespace node {

/**
 * @brief Overlay node. Calculates spatial location data on a set of ROIs on depth map.
 */
class Overlay : public DeviceNodeCRTP<DeviceNode, Overlay, OverlayProperties> {
   private:
    bool runOnHostVar = false;

   public:
    constexpr static const char* NAME = "Overlay";
    using DeviceNodeCRTP::DeviceNodeCRTP;
    Overlay() = default;
    Overlay(std::unique_ptr<Properties> props);

    /**
     * Input OverlayConfig message with ability to modify parameters in runtime.
     * Default queue is non-blocking with size 4.
     */
    Input inputFrame{*this, {"inputFrame", DEFAULT_GROUP, false, 4, {{{DatatypeEnum::ImgFrame, false}}}, DEFAULT_WAIT_FOR_MESSAGE}};

    /**
     * Input message with depth data used to retrieve spatial information about detected object.
     * Default queue is non-blocking with size 4.
     */
    Input inputDetections{*this, {"inputDetections", DEFAULT_GROUP, false, 4, {{{DatatypeEnum::ImgDetections, false}}}, DEFAULT_WAIT_FOR_MESSAGE}};

    /**
     * Outputs OverlayData message that carries spatial location results.
     */
    Output out{*this, {"out", DEFAULT_GROUP, {{{DatatypeEnum::ImgFrame, false}}}}};

    Overlay& setOverlayAlpha(float alpha);

    Overlay& setInterpolationType(int interpolationType);  // cv::INTER_LINEAR

    Overlay& setOutputSize(int width, int height);
};

}  // namespace node
}  // namespace dai
