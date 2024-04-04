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
    Properties& getProperties();

   public:
    PointCloud() = default;
    PointCloud(std::unique_ptr<Properties> props);
    /**
     * Initial config to use when computing the point cloud.
     */
    PointCloudConfig initialConfig;

    /**
     * Input PointCloudConfig message with ability to modify parameters in runtime.
     * Default queue is non-blocking with size 4.
     */
    Input inputConfig{true, *this, "inputConfig", Input::Type::SReceiver, false, 4, {{DatatypeEnum::PointCloudConfig, false}}};

    /**
     * Input message with depth data used to create the point cloud.
     * Default queue is non-blocking with size 4.
     */
    Input inputDepth{true, *this, "inputDepth", Input::Type::SReceiver, false, 4, true, {{DatatypeEnum::ImgFrame, false}}};

    /**
     * Outputs PointCloudData message
     */
    Output outputPointCloud{true, *this, "outputPointCloud", Output::Type::MSender, {{DatatypeEnum::PointCloudData, false}}};

    /**
     * Passthrough depth from which the point cloud was calculated.
     * Suitable for when input queue is set to non-blocking behavior.
     */
    Output passthroughDepth{true, *this, "passthroughDepth", Output::Type::MSender, {{DatatypeEnum::ImgFrame, false}}};

    /**
     * Specify number of frames in pool.
     * @param numFramesPool How many frames should the pool have
     */
    void setNumFramesPool(int numFramesPool);
};

}  // namespace node
}  // namespace dai
