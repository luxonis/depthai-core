#pragma once

#include <array>
#include <cstdint>
#include <string>
#include <vector>

#include "depthai/properties/Properties.hpp"

namespace dai {
namespace internal {

/**
 * Specify properties for PeerLinkIn, the consuming end of a direct device-to-device link
 */
struct PeerLinkInProperties : PropertiesSerializable<Properties, PeerLinkInProperties> {
    /**
     * Name of the stream to receive from the producer
     */
    std::string streamName;

    /**
     * Random value shared with the producing PeerLinkOut. It keeps links of different pipeline builds apart.
     */
    std::array<std::uint8_t, 16> token{};

    /**
     * Addresses of the producing device as "ip:port", tried in order
     */
    std::vector<std::string> producerEndpoints;

    /**
     * Longest wait between two connection attempts, in milliseconds
     */
    std::uint32_t reconnectIntervalMs = 1000;

    /**
     * Listener id of the producing device. An endpoint that answers with another id is another device that took
     * over the address, and the next endpoint is tried. Empty accepts any listener.
     */
    std::string producerListenerId;

    ~PeerLinkInProperties() override;
};

DEPTHAI_SERIALIZE_EXT_DECL(PeerLinkInProperties, streamName, token, producerEndpoints, reconnectIntervalMs, producerListenerId);

}  // namespace internal
}  // namespace dai

extern template struct dai::PropertiesSerializable<dai::Properties, dai::internal::PeerLinkInProperties>;
