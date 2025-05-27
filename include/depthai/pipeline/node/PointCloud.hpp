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
    /**
     * Initial config to use when computing the point cloud.
     */
    std::shared_ptr<PointCloudConfig> initialConfig = std::make_shared<PointCloudConfig>();

    /**
     * Input PointCloudConfig message with ability to modify parameters in runtime.
     * Default queue is non-blocking with size 4.
     */
    Input inputConfig{*this, {"inputConfig", DEFAULT_GROUP, false, 4, {{{DatatypeEnum::PointCloudConfig, false}}}, DEFAULT_WAIT_FOR_MESSAGE}};

    /**
     * Input message with depth data used to create the point cloud.
     * Default queue is non-blocking with size 4.
     */
    Input inputDepth{*this, {"inputDepth", DEFAULT_GROUP, false, 4, {{{DatatypeEnum::ImgFrame, false}}}, true}};

    /**
     * Outputs PointCloudData message
     */
    Output outputPointCloud{*this, {"outputPointCloud", DEFAULT_GROUP, {{{DatatypeEnum::PointCloudData, false}}}}};

    /**
     * Passthrough depth from which the point cloud was calculated.
     * Suitable for when input queue is set to non-blocking behavior.
     */
    Output passthroughDepth{*this, {"passthroughDepth", DEFAULT_GROUP, {{{DatatypeEnum::ImgFrame, false}}}}};
    /**
     * Specify number of frames in pool.
     * @param numFramesPool How many frames should the pool have
     */
    void setNumFramesPool(int numFramesPool);
};

}  // namespace node
}  // namespace dai
