#pragma once

#include <depthai/pipeline/DeviceNode.hpp>

// standard
#include <fstream>

// shared
#include <depthai/properties/PointCloudProperties.hpp>
#include <memory>

#include "depthai/pipeline/datatype/PointCloudConfig.hpp"

namespace dai {
namespace node {

/**
 * @brief PointCloud node. Computes point cloud from depth frames.
 */
class PointCloud : public DeviceNodeCRTP<DeviceNode, PointCloud, PointCloudProperties> {
   public:
    constexpr static const char* NAME = "PointCloud";

   protected:
    Properties& getProperties() override;
    using DeviceNodeCRTP::DeviceNodeCRTP;

   public:

    std::shared_ptr<PointCloud> build() {
        isBuild = true; 
        return std::static_pointer_cast<PointCloud>(shared_from_this());
    }

    /**
     * Initial config to use when computing the point cloud.
     */
    PointCloudConfig initialConfig;

    /**
     * Input PointCloudConfig message with ability to modify parameters in runtime.
     * Default queue is non-blocking with size 4.
     */
    Input inputConfig{*this, {.name = "inputConfig", .blocking = false, .queueSize = 4, .types = {{DatatypeEnum::PointCloudConfig, false}}}};

    /**
     * Input message with depth data used to create the point cloud.
     * Default queue is non-blocking with size 4.
     */
    Input inputDepth{*this, {.name = "inputDepth", .blocking = false, .queueSize = 4, .types = {{DatatypeEnum::ImgFrame, false}}, .waitForMessage = true}};

    /**
     * Outputs PointCloudData message
     */
    Output outputPointCloud{*this, {.name = "outputPointCloud", .types = {{DatatypeEnum::PointCloudData, false}}}};

    /**
     * Passthrough depth from which the point cloud was calculated.
     * Suitable for when input queue is set to non-blocking behavior.
     */
    Output passthroughDepth{*this, {.name = "passthroughDepth", .types = {{DatatypeEnum::ImgFrame, false}}}};
    /**
     * Specify number of frames in pool.
     * @param numFramesPool How many frames should the pool have
     */
    void setNumFramesPool(int numFramesPool);
};

}  // namespace node
}  // namespace dai
