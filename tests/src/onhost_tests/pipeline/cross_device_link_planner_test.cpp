// How Pipeline::build() carries a link between two devices (direct or relayed through the host), decided
// from what the host knows about the devices. No device needed.

#include <catch2/catch_all.hpp>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include "depthai/depthai.hpp"
#include "depthai/pipeline/node/internal/PeerLinkIn.hpp"
#include "depthai/pipeline/node/internal/PeerLinkOut.hpp"
#include "depthai/utility/Serialization.hpp"
#include "pipeline/CrossDeviceLinkPlanner.hpp"

using dai::CrossDeviceLinkDevice;
using dai::CrossDeviceLinkMode;
using dai::CrossDeviceLinkPlan;

namespace {

dai::peerlink::PeerLinkInfo peerLinkInfo(std::vector<std::string> addresses, std::string listenerId) {
    dai::peerlink::PeerLinkInfo info;
    info.protocolVersion = dai::peerlink::PEERLINK_PROTOCOL_VERSION;
    info.port = dai::peerlink::PEERLINK_DEFAULT_PORT;
    info.addresses = std::move(addresses);
    info.listenerId = std::move(listenerId);
    return info;
}

// An RVC4 that the host reaches over Ethernet at `hostAddress`
CrossDeviceLinkDevice rvc4(const std::string& id, const std::string& hostAddress, std::vector<std::string> addresses) {
    CrossDeviceLinkDevice device;
    device.deviceId = id;
    device.platform = dai::Platform::RVC4;
    device.hostAddress = hostAddress;
    device.peerLink = peerLinkInfo(std::move(addresses), id);
    return device;
}

// Probe that answers with `listenerId` at the given hosts and records what it was asked
struct FakeProbe {
    std::vector<std::pair<std::string, std::string>> answers;  // host -> listener id
    std::vector<std::string> asked;

    dai::CrossDeviceLinkProbe function() {
        return [this](const std::string& host, std::uint16_t port) -> std::optional<std::string> {
            asked.push_back(dai::peerlink::formatEndpoint(host, port));
            for(const auto& answer : answers) {
                if(answer.first == host) return answer.second;
            }
            return std::nullopt;
        };
    }
};

}  // namespace

TEST_CASE("Two reachable RVC4 devices get a direct link") {
    const auto producer = rvc4("A", "10.0.0.5", {"10.0.0.5"});
    const auto consumer = rvc4("B", "10.0.0.6", {"10.0.0.6"});
    FakeProbe probe{{{"10.0.0.5", "A"}}};
    const auto plan = dai::planCrossDeviceLink(CrossDeviceLinkMode::AUTO, producer, consumer, probe.function());
    REQUIRE(plan.direct);
    REQUIRE(plan.producerEndpoints == std::vector<std::string>{"10.0.0.5:11494"});
    REQUIRE(plan.producerListenerId == "A");
    REQUIRE(plan.reason.empty());
    // The host address and the reported one are the same address, probed once
    REQUIRE(probe.asked == std::vector<std::string>{"10.0.0.5:11494"});
}

TEST_CASE("HOST_RELAY never asks the devices") {
    const auto producer = rvc4("A", "10.0.0.5", {"10.0.0.5"});
    const auto consumer = rvc4("B", "10.0.0.6", {"10.0.0.6"});
    FakeProbe probe{{{"10.0.0.5", "A"}}};
    const auto plan = dai::planCrossDeviceLink(CrossDeviceLinkMode::HOST_RELAY, producer, consumer, probe.function());
    REQUIRE_FALSE(plan.direct);
    REQUIRE(plan.reason.find("HOST_RELAY") != std::string::npos);
    REQUIRE(probe.asked.empty());
}

TEST_CASE("An RVC2 device on either end gets the relay") {
    auto producer = rvc4("A", "10.0.0.5", {"10.0.0.5"});
    auto consumer = rvc4("B", "10.0.0.6", {"10.0.0.6"});
    FakeProbe probe{{{"10.0.0.5", "A"}}};
    SECTION("producer") {
        producer.platform = dai::Platform::RVC2;
        producer.peerLink.reset();
    }
    SECTION("consumer") {
        consumer.platform = dai::Platform::RVC2;
        consumer.peerLink.reset();
    }
    const auto plan = dai::planCrossDeviceLink(CrossDeviceLinkMode::AUTO, producer, consumer, probe.function());
    REQUIRE_FALSE(plan.direct);
    REQUIRE(plan.reason.find("RVC2") != std::string::npos);
    REQUIRE(probe.asked.empty());
}

TEST_CASE("Firmware without peer-link support gets the relay") {
    const auto producer = rvc4("A", "10.0.0.5", {"10.0.0.5"});
    auto consumer = rvc4("B", "10.0.0.6", {"10.0.0.6"});
    consumer.peerLink.reset();
    consumer.peerLinkError = "RPC 'getPeerLinkInfo' failed: Function not found";
    FakeProbe probe{{{"10.0.0.5", "A"}}};
    const auto plan = dai::planCrossDeviceLink(CrossDeviceLinkMode::AUTO, producer, consumer, probe.function());
    REQUIRE_FALSE(plan.direct);
    REQUIRE(plan.reason.find("device B firmware does not support direct links") != std::string::npos);
    REQUIRE(plan.reason.find("Function not found") != std::string::npos);
}

TEST_CASE("A peer-link protocol version mismatch gets the relay") {
    auto producer = rvc4("A", "10.0.0.5", {"10.0.0.5"});
    const auto consumer = rvc4("B", "10.0.0.6", {"10.0.0.6"});
    producer.peerLink->protocolVersion = dai::peerlink::PEERLINK_PROTOCOL_VERSION + 1;
    FakeProbe probe{{{"10.0.0.5", "A"}}};
    const auto plan = dai::planCrossDeviceLink(CrossDeviceLinkMode::AUTO, producer, consumer, probe.function());
    REQUIRE_FALSE(plan.direct);
    REQUIRE(plan.reason.find("protocol version") != std::string::npos);
}

TEST_CASE("An unreachable producer gets the relay") {
    const auto producer = rvc4("A", "10.0.0.5", {"10.0.0.5", "192.168.7.2"});
    const auto consumer = rvc4("B", "10.0.1.6", {"10.0.1.6"});
    FakeProbe probe;
    const auto plan = dai::planCrossDeviceLink(CrossDeviceLinkMode::AUTO, producer, consumer, probe.function());
    REQUIRE_FALSE(plan.direct);
    REQUIRE(plan.reason == "device B cannot reach device A at 10.0.0.5:11494, 192.168.7.2:11494");
}

TEST_CASE("Another device answering at the producer's address does not count") {
    // Device A reports an address of a network B is not on, where some other device has the same address
    const auto producer = rvc4("A", "", {"192.168.1.10"});
    const auto consumer = rvc4("B", "10.0.0.6", {"10.0.0.6"});
    FakeProbe probe{{{"192.168.1.10", "C"}}};
    const auto plan = dai::planCrossDeviceLink(CrossDeviceLinkMode::AUTO, producer, consumer, probe.function());
    REQUIRE_FALSE(plan.direct);
    REQUIRE(plan.reason.find("cannot reach") != std::string::npos);
}

TEST_CASE("A USB-connected producer is reached at the addresses it reports") {
    // The host has no IP of a USB device (or of the local device in standalone mode)
    const auto producer = rvc4("A", "", {"169.254.10.2", "10.0.0.5"});
    const auto consumer = rvc4("B", "10.0.0.6", {"10.0.0.6"});
    FakeProbe probe{{{"10.0.0.5", "A"}}};
    const auto plan = dai::planCrossDeviceLink(CrossDeviceLinkMode::AUTO, producer, consumer, probe.function());
    REQUIRE(plan.direct);
    REQUIRE(plan.producerEndpoints == std::vector<std::string>{"10.0.0.5:11494"});
    REQUIRE(probe.asked == std::vector<std::string>{"169.254.10.2:11494", "10.0.0.5:11494"});
}

TEST_CASE("A local device's socket path is not a network address") {
    // Standalone mode: the host reaches the local device over a socket, not an IP
    const auto producer = rvc4("A", "/tmp/xlink.sock", {"10.0.0.5"});
    const auto consumer = rvc4("B", "10.0.0.6", {"10.0.0.6"});
    FakeProbe probe{{{"10.0.0.5", "A"}}};
    const auto plan = dai::planCrossDeviceLink(CrossDeviceLinkMode::AUTO, producer, consumer, probe.function());
    REQUIRE(plan.direct);
    REQUIRE(probe.asked == std::vector<std::string>{"10.0.0.5:11494"});
}

TEST_CASE("A device name with the XLink port gives its host") {
    REQUIRE(dai::hostOfDeviceName("10.0.0.5:11491") == "10.0.0.5");
    REQUIRE(dai::hostOfDeviceName("10.0.0.5") == "10.0.0.5");
    REQUIRE(dai::hostOfDeviceName("[fe80::1]:11491") == "fe80::1");
    REQUIRE(dai::hostOfDeviceName("fe80::1") == "fe80::1");
    REQUIRE(dai::hostOfDeviceName("oak.local") == "oak.local");
}

TEST_CASE("A producer without any network address gets the relay") {
    const auto producer = rvc4("A", "", {});
    const auto consumer = rvc4("B", "10.0.0.6", {"10.0.0.6"});
    FakeProbe probe;
    const auto plan = dai::planCrossDeviceLink(CrossDeviceLinkMode::AUTO, producer, consumer, probe.function());
    REQUIRE_FALSE(plan.direct);
    REQUIRE(plan.reason == "device A reports no network address");
}

TEST_CASE("The host address comes first, all reached addresses are kept, loopback is skipped") {
    const auto producer = rvc4("A", "10.0.0.5", {"192.168.7.2", "10.0.0.5", "127.0.0.1"});
    const auto consumer = rvc4("B", "10.0.0.6", {"10.0.0.6"});
    FakeProbe probe{{{"10.0.0.5", "A"}, {"192.168.7.2", "A"}, {"127.0.0.1", "A"}}};
    const auto plan = dai::planCrossDeviceLink(CrossDeviceLinkMode::AUTO, producer, consumer, probe.function());
    REQUIRE(plan.direct);
    REQUIRE(plan.producerEndpoints == std::vector<std::string>{"10.0.0.5:11494", "192.168.7.2:11494"});
    REQUIRE(probe.asked == std::vector<std::string>{"10.0.0.5:11494", "192.168.7.2:11494"});

    REQUIRE(dai::isLoopbackPeerAddress("127.0.0.1"));
    REQUIRE(dai::isLoopbackPeerAddress("localhost"));
    REQUIRE(dai::isLoopbackPeerAddress("LocalHost"));
    REQUIRE(dai::isLoopbackPeerAddress("::1"));
    REQUIRE(dai::isLoopbackPeerAddress("[::1]"));
    REQUIRE_FALSE(dai::isLoopbackPeerAddress("10.0.0.127"));
}

TEST_CASE("DIRECT mode fails the build when a link cannot be direct") {
    const auto producer = rvc4("A", "10.0.0.5", {"10.0.0.5"});
    auto consumer = rvc4("B", "10.0.0.6", {"10.0.0.6"});
    FakeProbe reachable{{{"10.0.0.5", "A"}}};
    REQUIRE(dai::planCrossDeviceLink(CrossDeviceLinkMode::DIRECT, producer, consumer, reachable.function()).direct);

    FakeProbe unreachable;
    REQUIRE_THROWS_WITH(dai::planCrossDeviceLink(CrossDeviceLinkMode::DIRECT, producer, consumer, unreachable.function()),
                        Catch::Matchers::ContainsSubstring("cannot reach"));
    consumer.platform = dai::Platform::RVC2;
    REQUIRE_THROWS_WITH(dai::planCrossDeviceLink(CrossDeviceLinkMode::DIRECT, producer, consumer, reachable.function()),
                        Catch::Matchers::ContainsSubstring("RVC4"));
}

TEST_CASE("DEPTHAI_CROSS_DEVICE_LINK overrides the pipeline setting") {
    const bool setByUser = GENERATE(false, true);
    CAPTURE(setByUser);
    REQUIRE(dai::resolveCrossDeviceLinkMode(CrossDeviceLinkMode::AUTO, setByUser, "") == CrossDeviceLinkMode::AUTO);
    REQUIRE(dai::resolveCrossDeviceLinkMode(CrossDeviceLinkMode::DIRECT, setByUser, "") == CrossDeviceLinkMode::DIRECT);
    REQUIRE(dai::resolveCrossDeviceLinkMode(CrossDeviceLinkMode::AUTO, setByUser, "relay") == CrossDeviceLinkMode::HOST_RELAY);
    REQUIRE(dai::resolveCrossDeviceLinkMode(CrossDeviceLinkMode::HOST_RELAY, setByUser, "DIRECT") == CrossDeviceLinkMode::DIRECT);
    REQUIRE(dai::resolveCrossDeviceLinkMode(CrossDeviceLinkMode::DIRECT, setByUser, "Auto") == CrossDeviceLinkMode::AUTO);
    // Not a mode: the pipeline setting stays
    REQUIRE(dai::resolveCrossDeviceLinkMode(CrossDeviceLinkMode::DIRECT, setByUser, "sometimes") == CrossDeviceLinkMode::DIRECT);
}

TEST_CASE("Pipeline cross-device link mode defaults to AUTO and is fixed once built") {
    dai::Pipeline pipeline(false);
    REQUIRE(pipeline.getCrossDeviceLinkMode() == dai::Pipeline::CrossDeviceLinkMode::AUTO);
    pipeline.setCrossDeviceLinkMode(dai::CrossDeviceLinkMode::HOST_RELAY);
    REQUIRE(pipeline.getCrossDeviceLinkMode() == dai::CrossDeviceLinkMode::HOST_RELAY);
    pipeline.build();
    REQUIRE_THROWS(pipeline.setCrossDeviceLinkMode(dai::CrossDeviceLinkMode::DIRECT));
}

TEST_CASE("Peer-link info round trips through JSON") {
    const auto info = peerLinkInfo({"10.0.0.5", "192.168.7.2"}, "A");
    const auto parsed = dai::peerlink::peerLinkInfoFromJson(dai::peerlink::toJson(info));
    REQUIRE(parsed.protocolVersion == info.protocolVersion);
    REQUIRE(parsed.port == info.port);
    REQUIRE(parsed.addresses == info.addresses);
    REQUIRE(parsed.listenerId == info.listenerId);
    REQUIRE_THROWS_AS(dai::peerlink::peerLinkInfoFromJson(nlohmann::json{{"port", 1}}), std::runtime_error);
}

TEST_CASE("PeerLinkOut and PeerLinkIn properties round trip") {
    dai::internal::PeerLinkOutProperties out;
    out.streamName = "__p_12";
    for(std::size_t i = 0; i < out.token.size(); i++) out.token[i] = static_cast<std::uint8_t>(200 + i);
    out.maxFpsLimit = 12.5f;

    dai::internal::PeerLinkInProperties in;
    in.streamName = "__p_12";
    in.token = out.token;
    in.producerEndpoints = {"10.0.0.5:11494", "[fe80::1]:11494"};
    in.reconnectIntervalMs = 250;
    in.producerListenerId = "A";

    SECTION("libnop") {
        dai::internal::PeerLinkOutProperties outCopy;
        dai::utility::deserialize(dai::utility::serialize<dai::SerializationType::LIBNOP>(out), outCopy);
        REQUIRE(outCopy.streamName == out.streamName);
        REQUIRE(outCopy.token == out.token);
        REQUIRE(outCopy.maxFpsLimit == out.maxFpsLimit);

        dai::internal::PeerLinkInProperties inCopy;
        dai::utility::deserialize(dai::utility::serialize<dai::SerializationType::LIBNOP>(in), inCopy);
        REQUIRE(inCopy.streamName == in.streamName);
        REQUIRE(inCopy.token == in.token);
        REQUIRE(inCopy.producerEndpoints == in.producerEndpoints);
        REQUIRE(inCopy.reconnectIntervalMs == in.reconnectIntervalMs);
        REQUIRE(inCopy.producerListenerId == in.producerListenerId);
    }
    SECTION("JSON") {
        dai::internal::PeerLinkOutProperties outCopy;
        dai::utility::deserialize<dai::SerializationType::JSON>(dai::utility::serialize<dai::SerializationType::JSON>(out), outCopy);
        REQUIRE(outCopy.streamName == out.streamName);
        REQUIRE(outCopy.token == out.token);
        REQUIRE(outCopy.maxFpsLimit == out.maxFpsLimit);

        dai::internal::PeerLinkInProperties inCopy;
        dai::utility::deserialize<dai::SerializationType::JSON>(dai::utility::serialize<dai::SerializationType::JSON>(in), inCopy);
        REQUIRE(inCopy.producerEndpoints == in.producerEndpoints);
        REQUIRE(inCopy.token == in.token);
        REQUIRE(inCopy.reconnectIntervalMs == in.reconnectIntervalMs);
    }
}

TEST_CASE("PeerLinkOut and PeerLinkIn nodes keep their settings") {
    // Device nodes outside a pipeline: a host-only pipeline cannot hold them
    auto out = dai::node::internal::PeerLinkOut::create();
    out->setStreamName("__p_1");
    out->setFpsLimit(5.0f);
    auto in = dai::node::internal::PeerLinkIn::create();
    in->setStreamName("__p_1");
    in->setProducerEndpoints({"10.0.0.5:11494"});
    REQUIRE(out->getStreamName() == "__p_1");
    REQUIRE(out->getFpsLimit() == 5.0f);
    REQUIRE(in->getProducerEndpoints() == std::vector<std::string>{"10.0.0.5:11494"});
    // The producer side never blocks on a slow consumer
    REQUIRE_FALSE(out->input.getBlocking());
    REQUIRE(out->input.getMaxSize() == 8);
    REQUIRE(std::string(dai::node::internal::PeerLinkOut::NAME) == "PeerLinkOut");
    REQUIRE(std::string(dai::node::internal::PeerLinkIn::NAME) == "PeerLinkIn");
}
