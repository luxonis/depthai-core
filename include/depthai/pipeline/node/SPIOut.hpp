#pragma once

#include <depthai/pipeline/DeviceNode.hpp>

// shared
#include <depthai/properties/SPIOutProperties.hpp>

namespace dai {
namespace node {

/**
 * @brief SPIOut node. Sends messages over SPI.
 */
class SPIOut : public DeviceNodeCRTP<DeviceNode, SPIOut, SPIOutProperties> {
   public:
    constexpr static const char* NAME = "SPIOut";
    using DeviceNodeCRTP::DeviceNodeCRTP;
    void build() {
        properties.busId = 0;
    }

    /**
     * Input for any type of messages to be transferred over SPI stream
     * Default queue is blocking with size 8
     */
    Input input{*this, {.name = "in", .blocking = true, .queueSize = 8, .types = {{DatatypeEnum::Buffer, true}}, .waitForMessage = true}};

    /**
     * Specifies stream name over which the node will send data
     *
     * @param name Stream name
     */
    void setStreamName(std::string name) {
        properties.streamName = name;
    }

    /**
     * Specifies SPI Bus number to use
     * @param id SPI Bus id
     */
    void setBusId(int busId) {
        properties.busId = busId;
    }
};

}  // namespace node
}  // namespace dai
