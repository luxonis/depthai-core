#pragma once

#include <depthai/pipeline/DeviceNode.hpp>

// standard
#include <fstream>

// shared
#include <depthai-shared/properties/PointCloudProperties.hpp>

#include "depthai/pipeline/datatype/PointCloudConfig.hpp"
#include "depthai/pipeline/datatype/OccupancyPool.hpp"

namespace dai {
namespace node {

/**
 * @brief PointCloud node.
 */
class PointCloud : public NodeCRTP<DeviceNode, PointCloud, PointCloudProperties> {
   public:
    constexpr static const char* NAME = "PointCloud";
    using NodeCRTP::NodeCRTP;

   protected:
    Properties& getProperties();

   private:
    std::shared_ptr<RawPointCloudConfig> rawConfig;

   public:
    PointCloud();
    PointCloud(std::unique_ptr<Properties> props);

    /**
     * Initial config to use when calculating spatial location data.
     */
    PointCloudConfig initialConfig;

    /**
     * Input PointCloudConfig message with ability to modify parameters in runtime.
     * Default queue is non-blocking with size 4.
     */
    Input inputConfig{true, *this, "inputConfig", Input::Type::SReceiver, false, 4, {{DatatypeEnum::PointCloudConfig, false}}};
    /**
     * Input message with depth data used to retrieve spatial information about detected object.
     * Default queue is non-blocking with size 4.
     */
    Input inputDepth{true, *this, "inputDepth", Input::Type::SReceiver, false, 4, true, {{DatatypeEnum::ImgFrame, false}}};

    /**
     * Outputs ImgFrame message that carries spatial location results.
     */
    Output outputPointCloud{true, *this, "outputPointCloud", Output::Type::MSender, {{DatatypeEnum::ImgFrame, false}}};

    /**
     * Outputs occupancy pool message.
     */
    Output outputOccupancyPool{true, *this, "outputOccupancyPool", Output::Type::MSender, {{DatatypeEnum::OccupancyPool, false}}};

    /**
     * Passthrough message on which the calculation was performed.
     * Suitable for when input queue is set to non-blocking behavior.
     */
    Output passthroughDepth{true, *this, "passthroughDepth", Output::Type::MSender, {{DatatypeEnum::ImgFrame, false}}};

    // Functions to set properties
    /**
     * Specify whether or not wait until configuration message arrives to inputConfig Input.
     * @param wait True to wait for configuration message, false otherwise.
     */
    [[deprecated("Use 'inputConfig.setWaitForMessage()' instead")]] void setWaitForConfigInput(bool wait);

    /**
     * @see setWaitForConfigInput
     * @returns True if wait for inputConfig message, false otherwise
     */
    [[deprecated("Use 'inputConfig.setWaitForMessage()' instead")]] bool getWaitForConfigInput() const;
};

}  // namespace node
}  // namespace dai
