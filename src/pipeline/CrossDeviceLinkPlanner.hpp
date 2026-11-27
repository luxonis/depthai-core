#pragma once

// Decides how Pipeline::build() carries a link between nodes on two different devices: directly between the
// devices (PeerLinkOut -> PeerLinkIn) or relayed through the host (XLinkOut -> host -> XLinkIn).
// The decision is a pure function of what the host knows about the devices, so it is tested without devices.

#include <cstdint>
#include <functional>
#include <optional>
#include <string>
#include <vector>

#include "depthai/device/Platform.hpp"
#include "depthai/pipeline/Pipeline.hpp"
#include "peerlink/PeerLinkProtocol.hpp"

namespace dai {

/// Environment variable that overrides Pipeline::setCrossDeviceLinkMode()
constexpr const char* CROSS_DEVICE_LINK_ENV = "DEPTHAI_CROSS_DEVICE_LINK";

/// What the host knows about one device of a cross-device link
struct CrossDeviceLinkDevice {
    std::string deviceId;
    Platform platform = Platform::RVC4;
    /// Address the host connected to the device at; empty when the host link is not TCP/IP
    std::string hostAddress;
    /// Reported by the device firmware; std::nullopt when the firmware does not support direct links
    std::optional<peerlink::PeerLinkInfo> peerLink;
    /// Why peerLink is missing
    std::string peerLinkError;
};

struct CrossDeviceLinkPlan {
    bool direct = false;
    /// Producer addresses that the consumer reached, for PeerLinkIn
    std::vector<std::string> producerEndpoints;
    /// Listener id of the producer, so PeerLinkIn can tell the producer from a device that takes over its address
    std::string producerListenerId;
    /// Why the link is relayed through the host
    std::string reason;
};

/// Asks the consuming device whether a peer-link listener answers at host:port; returns its listener id
using CrossDeviceLinkProbe = std::function<std::optional<std::string>(const std::string& host, std::uint16_t port)>;

/// Parses "auto", "direct" or "relay" (any case); std::nullopt otherwise
std::optional<CrossDeviceLinkMode> parseCrossDeviceLinkMode(const std::string& value);
const char* crossDeviceLinkModeName(CrossDeviceLinkMode mode);

/**
 * The mode from DEPTHAI_CROSS_DEVICE_LINK when it is set to a valid value, `apiMode` otherwise. Logs a
 * warning when the variable overrides a mode set with Pipeline::setCrossDeviceLinkMode() or is not valid.
 * @param apiSetByUser Whether `apiMode` was set with Pipeline::setCrossDeviceLinkMode()
 * @param envValue Value of DEPTHAI_CROSS_DEVICE_LINK, empty when not set
 */
CrossDeviceLinkMode resolveCrossDeviceLinkMode(CrossDeviceLinkMode apiMode, bool apiSetByUser, const std::string& envValue);

/// True for "localhost", 127.0.0.0/8 and "::1" (also in brackets), in any case
bool isLoopbackPeerAddress(const std::string& host);

/// The host of a TCP device name, which may carry the XLink port ("10.0.0.5:11491" -> "10.0.0.5")
std::string hostOfDeviceName(const std::string& name);

/**
 * Decides how one link from `producer` to `consumer` is carried. In AUTO mode the link is direct when both
 * devices are RVC4, both firmwares support peer links with this protocol version, and the consumer reaches
 * the producer at one of its addresses: the address the host uses first, then the ones the producer reports.
 * Loopback addresses and socket paths (a local device) are skipped.
 * @throws std::runtime_error in DIRECT mode when the link cannot be direct
 */
CrossDeviceLinkPlan planCrossDeviceLink(CrossDeviceLinkMode mode,
                                        const CrossDeviceLinkDevice& producer,
                                        const CrossDeviceLinkDevice& consumer,
                                        const CrossDeviceLinkProbe& probe);

}  // namespace dai
