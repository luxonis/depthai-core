#pragma once

#include <depthai/pipeline/DeviceNodeGroup.hpp>
#include <depthai/pipeline/Subnode.hpp>
#include <depthai/pipeline/node/CropConfigGenerator.hpp>
#include <depthai/pipeline/node/ImageManip.hpp>

namespace dai {
namespace node {

/**
 * @brief Crops every detection region from its associated image.
 *
 * This node group combines CropConfigGenerator and ImageManip. For every
 * detection in inputDetections, one cropped ImgFrame is emitted on out.
 * ImgDetections, SpatialImgDetections, and Tracklets are supported.
 *
 * The internal ImageManip waits for each generated config, ensuring that the
 * config is applied to the image emitted with it.
 */
class DetectionsCrop : public DeviceNodeGroup, public HostRunnable {
   private:
    bool runOnHostVar = false;

   public:
    DetectionsCrop();
    explicit DetectionsCrop(const std::shared_ptr<Device>& device);
    ~DetectionsCrop() override;

    [[nodiscard]] static std::shared_ptr<DetectionsCrop> create();
    [[nodiscard]] static std::shared_ptr<DetectionsCrop> create(const std::shared_ptr<Device>& device);

    Subnode<CropConfigGenerator> cropConfigGenerator{*this, "cropConfigGenerator"};
    Subnode<ImageManip> imageManip{*this, "imageManip"};

    /**
     * Input detection regions. Accepts ImgDetections (including
     * SpatialImgDetections) and Tracklets.
     */
    Input& inputDetections;

    /**
     * Input image associated with inputDetections.
     */
    Input& inputImage;

    /**
     * Cropped images, emitted one by one in detection order.
     */
    Output& out;

    /**
     * Specify whether the group and both internal nodes run on host or device.
     */
    DetectionsCrop& setRunOnHost(bool runOnHost = true);

    /**
     * Check whether the group is configured to run on host.
     */
    bool runOnHost() const override;

    void buildInternal() override;
};

}  // namespace node
}  // namespace dai
