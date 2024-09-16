#pragma once

#include <depthai/pipeline/DeviceNode.hpp>

// standard
#include <fstream>

// shared
#include <depthai-shared/properties/VisionHealthProperties.hpp>

#include "depthai/pipeline/datatype/VisionHealthConfig.hpp"
#include "depthai/pipeline/datatype/VisionHealthMetrics.hpp"

namespace dai {
namespace node {

/**
 * @brief VisionHealth node.
 */
class VisionHealth : public NodeCRTP<DeviceNode, VisionHealth, VisionHealthProperties> {
   public:
    constexpr static const char* NAME = "VisionHealth";
    using NodeCRTP::NodeCRTP;

   protected:
    Properties& getProperties();

   private:
    std::shared_ptr<RawVisionHealthConfig> rawConfig;

   public:
    VisionHealth();
    VisionHealth(std::unique_ptr<Properties> props);

    /**
     * Initial config to use when calculating spatial location data.
     */
    VisionHealthConfig initialConfig;

    /**
     * Input message with depth data used to retrieve spatial information about detected object.
     * Default queue is non-blocking with size 4.
     */
    Input inputImage{true, *this, "inputImage", Input::Type::SReceiver, false, 4, true, {{DatatypeEnum::ImgFrame, false}}};

    /**
     * Outputs ImgFrame message that carries spatial location results.
     */
    Output outputMetrics{true, *this, "outputMetrics", Output::Type::MSender, {{DatatypeEnum::VisionHealthMetrics, false}}};

    /**
     * Passthrough message on which the calculation was performed.
     * Suitable for when input queue is set to non-blocking behavior.
     */
    Output passthroughInputImage{true, *this, "passthroughInputImage", Output::Type::MSender, {{DatatypeEnum::ImgFrame, false}}};
};

}  // namespace node
}  // namespace dai
