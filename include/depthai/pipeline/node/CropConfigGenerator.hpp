#pragma once

#include <depthai/pipeline/DeviceNode.hpp>

#include "depthai/pipeline/datatype/ImageManipConfig.hpp"
#include "depthai/pipeline/datatype/ImgDetections.hpp"
#include "depthai/pipeline/datatype/ImgFrame.hpp"
#include "depthai/pipeline/datatype/SpatialImgDetections.hpp"
#include "depthai/pipeline/datatype/Tracklets.hpp"
#include "depthai/properties/CropConfigGeneratorProperties.hpp"

namespace dai {
namespace node {

/**
 * @brief Generates an ImageManip crop request for every detection in a frame.
 *
 * Each detection produces one ImageManipConfig on `outConfig` followed by the
 * corresponding ImgFrame on `outImage`. Link both outputs to the respective
 * ImageManip inputs. The generated crop preserves rotated detection boxes.
 *
 * ImgDetections, SpatialImgDetections, and Tracklets are supported. When both
 * the detection message and image contain valid transformation metadata, the
 * detection regions are remapped to the input image before configs are emitted.
 */
class CropConfigGenerator : public DeviceNodeCRTP<DeviceNode, CropConfigGenerator, CropConfigGeneratorProperties>, public HostRunnable {
   private:
    bool runOnHostVar = false;

   public:
    constexpr static const char* NAME = "CropConfigGenerator";
    using DeviceNodeCRTP::DeviceNodeCRTP;

    /**
     * Input detection regions. Accepts ImgDetections (including
     * SpatialImgDetections) and Tracklets.
     */
    Input inputDetections{*this,
                          {"inputDetections",
                           DEFAULT_GROUP,
                           DEFAULT_BLOCKING,
                           DEFAULT_QUEUE_SIZE,
                           {{{DatatypeEnum::ImgDetections, true}, {DatatypeEnum::Tracklets, false}}},
                           DEFAULT_WAIT_FOR_MESSAGE}};

    /**
     * Input image associated with inputDetections.
     */
    Input inputImage{*this, {"inputImage", DEFAULT_GROUP, DEFAULT_BLOCKING, DEFAULT_QUEUE_SIZE, {{{DatatypeEnum::ImgFrame, false}}}, DEFAULT_WAIT_FOR_MESSAGE}};

    /**
     * One crop config for every detection. Link to ImageManip::inputConfig.
     */
    Output outConfig{*this, {"outConfig", DEFAULT_GROUP, {{{DatatypeEnum::ImageManipConfig, false}}}}};

    /**
     * The associated image, repeated once per detection. Link to
     * ImageManip::inputImage.
     */
    Output outImage{*this, {"outImage", DEFAULT_GROUP, {{{DatatypeEnum::ImgFrame, false}}}}};

    /**
     * Specify whether to run on host or device. By default, the node runs on
     * device.
     */
    void setRunOnHost(bool runOnHost);

    /**
     * Check whether the node is configured to run on host.
     */
    bool runOnHost() const override;

    void run() override;
};

}  // namespace node
}  // namespace dai
