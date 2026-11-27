#pragma once

#include <array>
#include <cstdint>
#include <string>

#include "depthai/properties/Properties.hpp"

namespace dai {
namespace internal {

/**
 * Specify properties for PeerLinkOut, the producing end of a direct device-to-device link
 */
struct PeerLinkOutProperties : PropertiesSerializable<Properties, PeerLinkOutProperties> {
    /**
     * Name of the stream the consumer asks for
     */
    std::string streamName;

    /**
     * Random value shared with the consuming PeerLinkIn. It keeps links of different pipeline builds apart.
     */
    std::array<std::uint8_t, 16> token{};

    /**
     * Set a limit to how many messages are sent per second (-1 = unlimited)
     */
    float maxFpsLimit = -1;

    ~PeerLinkOutProperties() override;
};

DEPTHAI_SERIALIZE_EXT_DECL(PeerLinkOutProperties, streamName, token, maxFpsLimit);

}  // namespace internal
}  // namespace dai

extern template struct dai::PropertiesSerializable<dai::Properties, dai::internal::PeerLinkOutProperties>;
