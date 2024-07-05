#pragma once

#include <depthai/pipeline/DeviceNode.hpp>

// shared
#include <depthai/properties/SPIInProperties.hpp>

namespace dai {
namespace node {

/**
 * @brief SPIIn node. Receives messages over SPI.
 */
class SPIIn : public DeviceNodeCRTP<DeviceNode, SPIIn, SPIInProperties> {
   public:
    constexpr static const char* NAME = "SPIIn";
    using DeviceNodeCRTP::DeviceNodeCRTP;
    std::shared_ptr<SPIIn> build();

    /**
     * Outputs message of same type as send from host.
     */
    Output out{*this, {"out", DEFAULT_GROUP, {{{DatatypeEnum::Buffer, true}}}}};
    /**
     * Specifies stream name over which the node will receive data
     *
     * @param name Stream name
     */
    void setStreamName(const std::string& name);

    /**
     * Specifies SPI Bus number to use
     * @param id SPI Bus id
     */
    void setBusId(int id);

    /**
     * Set maximum message size it can receive
     * @param maxDataSize Maximum size in bytes
     */
    void setMaxDataSize(std::uint32_t maxDataSize);

    /**
     * Set number of frames in pool for sending messages forward
     * @param numFrames Maximum number of frames in pool
     */
    void setNumFrames(std::uint32_t numFrames);

    /// Get stream name
    std::string getStreamName() const;
    /// Get bus id
    int getBusId() const;
    /// Get maximum messages size in bytes
    std::uint32_t getMaxDataSize() const;
    /// Get number of frames in pool
    std::uint32_t getNumFrames() const;

   protected:
    bool isBuild = false;
    bool needsBuild() override {
        return !isBuild;
    }
};

}  // namespace node
}  // namespace dai
