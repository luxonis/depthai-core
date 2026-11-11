#include <catch2/catch_all.hpp>
#include <chrono>
#include <memory>
#include <string>
#include <tuple>
#include <vector>

#include "depthai/depthai.hpp"
#include "depthai/device/DeviceGate.hpp"
#include "depthai/pipeline/PipelineSchema.hpp"
#include "depthai/xlink/XLinkConnection.hpp"
#include "depthai/xlink/XLinkConstants.hpp"
#include "depthai/xlink/XLinkStream.hpp"
#include "nanorpc/core/client.h"
#include "nanorpc/packer/nlohmann_msgpack.h"

namespace {

// DeviceGate does not end its session on destruction, so a failed assertion would leave the device busy
struct SessionGuard {
    dai::DeviceGate& gate;
    ~SessionGuard() {
        gate.destroySession();
    }
};

}  // namespace

TEST_CASE("Gate session answers raw RPC before timesync and names an unknown node") {
    using namespace std::chrono_literals;
    const auto [found, info] = dai::Device::getAnyAvailableDevice();
    REQUIRE(found);
    REQUIRE(info.state == X_LINK_GATE);

    dai::DeviceGate gate(info);
    REQUIRE(gate.createSession());
    SessionGuard guard{gate};
    REQUIRE(gate.startSession());
    {
        auto connection = std::make_shared<dai::XLinkConnection>(info, X_LINK_ANY_STATE);
        dai::XLinkStream rpc(connection, dai::device::XLINK_CHANNEL_MAIN_RPC, dai::device::XLINK_USB_BUFFER_MAX_SIZE);
        dai::XLinkStream telemetry(connection, dai::device::XLINK_CHANNEL_TELEMETRY, dai::device::XLINK_USB_BUFFER_MAX_SIZE);
        nanorpc::core::client<nanorpc::packer::nlohmann_msgpack> client([&rpc](nanorpc::core::type::buffer request) {
            rpc.write(request);
            return rpc.read(2s);
        });

        // The client appends the method name itself; the server's own text is what proves the lookup failed there
        REQUIRE_THROWS_MATCHES(client.call("testMissingMethod"),
                               nanorpc::core::exception::logic,
                               Catch::Matchers::MessageMatches(Catch::Matchers::ContainsSubstring("Function not found")));

        // An unparsable request gets an error reply: protocol 1, response type 2, status fail 0, message
        for(const auto& request : {std::vector<uint8_t>{0x90}, std::vector<uint8_t>{0x94}}) {
            rpc.write(request);
            const auto reply = nlohmann::json::from_msgpack(rpc.read(2s));
            REQUIRE(reply.is_array());
            REQUIRE(reply.size() == 4);
            REQUIRE(reply[0] == 1);
            REQUIRE(reply[1] == 2);
            REQUIRE(reply[2] == 0);
            REQUIRE(reply[3].is_string());
            REQUIRE_FALSE(reply[3].get<std::string>().empty());
        }

        const auto identity = client.call("getMxId").as<std::tuple<bool, std::string, std::string>>();
        REQUIRE(std::get<0>(identity));
        REQUIRE(std::get<2>(identity) == info.getDeviceId());

        dai::PipelineSchema schema;
        schema.nodes[0].id = 0;
        schema.nodes[0].name = "TestMissingNode";
        const auto accepted = client.call("setPipelineSchema", schema).as<std::tuple<bool, std::string>>();
        REQUIRE(std::get<0>(accepted));
        const auto built = client.call("buildPipeline").as<std::tuple<bool, std::string, std::tuple<bool, std::string>>>();
        REQUIRE(std::get<0>(built));
        REQUIRE_FALSE(std::get<0>(std::get<2>(built)));
        REQUIRE_THAT(std::get<1>(std::get<2>(built)), Catch::Matchers::ContainsSubstring("TestMissingNode"));
        connection->close();
    }
    REQUIRE(gate.destroySession());
}
