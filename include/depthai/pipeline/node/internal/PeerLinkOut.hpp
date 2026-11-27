#pragma once

#include <depthai/pipeline/DeviceNode.hpp>

// shared
#include <depthai/properties/internal/PeerLinkOutProperties.hpp>

namespace dai {
namespace node {
namespace internal {

using PeerLinkOutProperties = ::dai::internal::PeerLinkOutProperties;

/**
 * @brief PeerLinkOut node. Sends messages directly to a PeerLinkIn node on another device, without the host.
 *
 * Pipeline::build() inserts it for a link between two devices, see Pipeline::setCrossDeviceLinkMode().
 * While no consumer is connected, messages are dropped.
 */
class PeerLinkOut : public DeviceNodeCRTP<DeviceNode, PeerLinkOut, PeerLinkOutProperties> {
   public:
    constexpr static const char* NAME = "PeerLinkOut";
    using DeviceNodeCRTP::DeviceNodeCRTP;

    /**
     * Input for any type of messages to be sent to the other device.
     *
     * Default queue is non-blocking with size 8, so a slow consumer never stalls the producing device
     */
    Input input{*this, {"in", DEFAULT_GROUP, false, 8, {{{DatatypeEnum::Buffer, true}}}, DEFAULT_WAIT_FOR_MESSAGE}};

    /**
     * Specifies the stream name the consuming PeerLinkIn asks for
     * @param name Stream name
     */
    void setStreamName(const std::string& name);

    /**
     * Specifies a message sending limit. It's approximated from specified rate.
     *
     * @param fps Approximate rate limit in messages per second
     */
    void setFpsLimit(float fps);

    /// Get stream name
    std::string getStreamName() const;
    /// Get rate limit in messages per second
    float getFpsLimit() const;
};

}  // namespace internal
}  // namespace node
}  // namespace dai
