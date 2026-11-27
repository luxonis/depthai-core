#include <catch2/catch_all.hpp>
#include <cstdint>
#include <string>

#include "device/DeviceRpc.hpp"
#include "nanorpc/core/server.h"
#include "nanorpc/packer/nlohmann_msgpack.h"

TEST_CASE("Device RPC keeps a uint64 payload separate from the method ID", "[rpc]") {
    using Packer = nanorpc::packer::nlohmann_msgpack;
    nanorpc::core::server<Packer> server;
    std::uint64_t stored = 0;
    server.handle("replayCacheSetLimit", [&](std::uint64_t value) {
        stored = value;
        return true;
    });
    nanorpc::core::client<Packer> client([&](auto request) { return server.execute(std::move(request)); });
    const auto limit = GENERATE(std::uint64_t{0}, std::uint64_t{16} << 30, UINT64_MAX);
    REQUIRE(dai::detail::callDeviceRpc(client, "replayCacheSetLimit", limit).as<bool>());
    REQUIRE(stored == limit);
}

TEST_CASE("Device RPC preserves names and argument lists", "[rpc]") {
    using Packer = nanorpc::packer::nlohmann_msgpack;
    nanorpc::core::server<Packer> server;
    server.handle("get", [] { return 7; });
    server.handle("write", [](std::uint64_t offset, std::string text) { return text + std::to_string(offset); });
    server.handle("name", [](std::string text) { return text; });
    nanorpc::core::client<Packer> client([&](auto request) { return server.execute(std::move(request)); });
    REQUIRE(dai::detail::callDeviceRpc(client, "get").as<int>() == 7);
    REQUIRE(dai::detail::callDeviceRpc(client, "write", std::uint64_t{42}, std::string("offset=")).as<std::string>() == "offset=42");
    REQUIRE(dai::detail::callDeviceRpc(client, "name", std::string("source")).as<std::string>() == "source");
}
