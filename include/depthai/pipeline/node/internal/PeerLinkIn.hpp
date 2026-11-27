#pragma once

#include <depthai/pipeline/DeviceNode.hpp>

// shared
#include <depthai/properties/internal/PeerLinkInProperties.hpp>

namespace dai {
namespace node {
namespace internal {

using PeerLinkInProperties = ::dai::internal::PeerLinkInProperties;

/**
 * @brief PeerLinkIn node. Receives messages directly from a PeerLinkOut node on another device, without the host.
 *
 * Pipeline::build() inserts it for a link between two devices, see Pipeline::setCrossDeviceLinkMode().
 * It connects to the producing device and reconnects by itself when the link is lost.
 */
class PeerLinkIn : public DeviceNodeCRTP<DeviceNode, PeerLinkIn, PeerLinkInProperties> {
   public:
    constexpr static const char* NAME = "PeerLinkIn";
    using DeviceNodeCRTP::DeviceNodeCRTP;

    /**
     * Outputs the messages received from the other device
     */
    Output out{*this, {"out", DEFAULT_GROUP, {{{DatatypeEnum::Buffer, true}}}}};

    /**
     * Specifies the stream name to receive from the producing PeerLinkOut
     * @param name Stream name
     */
    void setStreamName(const std::string& name);

    /**
     * Specifies the addresses of the producing device, tried in order
     * @param endpoints Addresses as "ip:port"
     */
    void setProducerEndpoints(const std::vector<std::string>& endpoints);

    /// Get stream name
    std::string getStreamName() const;
    /// Get the addresses of the producing device
    std::vector<std::string> getProducerEndpoints() const;
};

}  // namespace internal
}  // namespace node
}  // namespace dai
