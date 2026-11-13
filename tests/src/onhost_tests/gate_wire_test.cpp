#include <catch2/catch_test_macros.hpp>
#include <cstdint>
#include <string>
#include <vector>

#include "device/GateWire.hpp"

using dai::gatewire::sha256Hex;

namespace {
std::vector<uint8_t> bytes(const std::string& text) {
    return {text.begin(), text.end()};
}
}  // namespace

TEST_CASE("sha256Hex returns 64 lowercase hex digits") {
    const auto empty = sha256Hex({});
    if(empty.empty()) {
        SKIP("built without a sha256 implementation");
    }
    REQUIRE(empty == "e3b0c44298fc1c149afbf4c8996fb92427ae41e4649b934ca495991b7852b855");
    REQUIRE(sha256Hex(bytes("abc")) == "ba7816bf8f01cfea414140de5dae2223b00361a396177a9cb410ff61f20015ad");
}
