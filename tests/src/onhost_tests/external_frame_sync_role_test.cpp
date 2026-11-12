#include <catch2/catch_all.hpp>
#include <cstdint>
#include <nlohmann/json.hpp>
#include <string>
#include <tuple>

#include "depthai/common/ExternalFrameSyncRoles.hpp"
#include "nanorpc/packer/nlohmann_msgpack.h"

using dai::ExternalFrameSyncRole;

// The device firmware is a separate build that reads the role from the RPC payload as a number.
TEST_CASE("External frame sync roles keep their RPC values and names") {
    const auto [role, rpcValue, name] = GENERATE(table<ExternalFrameSyncRole, int32_t, std::string>({{ExternalFrameSyncRole::AUTO_DETECT, -1, "AUTO DETECT"},
                                                                                                     {ExternalFrameSyncRole::MASTER, 0, "MASTER"},
                                                                                                     {ExternalFrameSyncRole::SLAVE, 1, "SLAVE"},
                                                                                                     {ExternalFrameSyncRole::STANDALONE, 2, "STANDALONE"}}));
    CAPTURE(name);

    nanorpc::packer::nlohmann_msgpack packer;
    const auto buffer = packer.pack(std::make_tuple(role)).to_buffer();
    REQUIRE(nlohmann::json::from_msgpack(buffer) == nlohmann::json::array({nlohmann::json::array({rpcValue})}));

    std::tuple<ExternalFrameSyncRole> unpacked{};
    packer.from_buffer(buffer).unpack(unpacked);
    REQUIRE(std::get<0>(unpacked) == role);

    REQUIRE(dai::toString(role) == name);
}
