#pragma once

#include <depthai/pipeline/Node.hpp>

// standard
#include <fstream>

// shared
#include <depthai-shared/properties/PointCloudProperties.hpp>

#include "depthai/pipeline/datatype/PointCloudConfig.hpp"

namespace dai {
namespace node {

/**
 * @brief PointCloud node. Computes point cloud from depth frames.
 */
class PointCloud : public NodeCRTP<Node, PointCloud, PointCloudProperties> {
   public:
    constexpr static const char* NAME = "PointCloud";

   protected:
    Properties& getProperties();

   private:
    std::shared_ptr<RawPointCloudConfig> rawConfig;

   public:
    PointCloud(const std::shared_ptr<PipelineImpl>& par, int64_t nodeId);
    PointCloud(const std::shared_ptr<PipelineImpl>& par, int64_t nodeId, std::unique_ptr<Properties> props);

    /**
     * Initial config to use when computing the point cloud.
     */
    PointCloudConfig initialConfig;

    /**
     * Input PointCloudConfig message with ability to modify parameters in runtime.
     * Default queue is non-blocking with size 4.
     */
    Input inputConfig{*this, "inputConfig", Input::Type::SReceiver, false, 4, {{DatatypeEnum::PointCloudConfig, false}}};

    /**
     * Input message with depth data used to create the point cloud.
     * Default queue is non-blocking with size 4.
     */
    Input inputDepth{*this, "inputDepth", Input::Type::SReceiver, false, 4, true, {{DatatypeEnum::ImgFrame, false}}};

    /**
     * Outputs PointCloudData message
     */
    Output outputPointCloud{*this, "outputPointCloud", Output::Type::MSender, {{DatatypeEnum::PointCloudData, false}}};

    /**
     * Passthrough depth from which the point cloud was calculated.
     * Suitable for when input queue is set to non-blocking behavior.
     */
    Output passthroughDepth{*this, "passthroughDepth", Output::Type::MSender, {{DatatypeEnum::ImgFrame, false}}};

    /**
     * Specify number of frames in pool.
     * @param numFramesPool How many frames should the pool have
     */
    void setNumFramesPool(int numFramesPool);
};

}  // namespace node
}  // namespace dai
