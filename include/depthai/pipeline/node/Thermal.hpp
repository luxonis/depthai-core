#pragma once

#include <depthai/pipeline/DeviceNode.hpp>

// standard
#include <fstream>

// shared
#include <depthai/properties/ThermalProperties.hpp>

#include "depthai/pipeline/datatype/ThermalConfig.hpp"

namespace dai {
namespace node {

/**
 * @brief Thermal node.
 */
class Thermal : public DeviceNodeCRTP<DeviceNode, Thermal, ThermalProperties> {
   public:
    constexpr static const char* NAME = "Thermal";
    using DeviceNodeCRTP::DeviceNodeCRTP;

   protected:
    Properties& getProperties();

   public:
    Thermal() = default;
    /**
     * Construct a Thermal node with properties.
     */
    Thermal(std::unique_ptr<Properties> props);

    /**
     * Initial config to use for thermal sensor.
     */
    std::shared_ptr<ThermalConfig> initialConfig = std::make_shared<ThermalConfig>();

    /**
     * Input ThermalConfig message with ability to modify parameters in runtime.
     * Default queue is non-blocking with size 4.
     */
    Input inputConfig{*this,
                      {"inputConfig", DEFAULT_GROUP, DEFAULT_BLOCKING, DEFAULT_QUEUE_SIZE, {{{DatatypeEnum::ThermalConfig, false}}}, DEFAULT_WAIT_FOR_MESSAGE}};

    /**
     * Outputs FP16 (degC) thermal image.
     */
    Output temperature{*this, {"temperature", DEFAULT_GROUP, {{{DatatypeEnum::ImgFrame, false}}}}};

    /**
     * Outputs YUV422i grayscale thermal image.
     */
    Output color{*this, {"color", DEFAULT_GROUP, {{{DatatypeEnum::ImgFrame, false}}}}};

    /**
     * Build with a specific board socket and fps.
     */
    std::shared_ptr<Thermal> build(dai::CameraBoardSocket boardSocket = dai::CameraBoardSocket::AUTO, float fps = 25);

    /**
     * Retrieves which board socket to use
     * @returns Board socket to use
     */
    CameraBoardSocket getBoardSocket() const;

    /**
     * Set output frames per second.
     */
    void setFps(float fps);

   private:
    bool isBuilt = false;
};

}  // namespace node
}  // namespace dai
