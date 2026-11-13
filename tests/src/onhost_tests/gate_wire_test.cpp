#include <catch2/catch_test_macros.hpp>
#include <cstdint>
#include <string>
#include <vector>

#include "device/GateWire.hpp"

using dai::gatewire::parseRawFile;
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

TEST_CASE("parseRawFile splits the filename and the file bytes") {
    std::string filename = "unchanged";

    SECTION("a file after a filename") {
        auto file = parseRawFile({5, 0, 0, 0, 'a', '.', 't', 'a', 'r', 1, 2, 3}, filename);
        REQUIRE(file.has_value());
        REQUIRE(*file == std::vector<uint8_t>{1, 2, 3});
        REQUIRE(filename == "a.tar");
    }

    SECTION("the length is little-endian") {
        std::vector<uint8_t> payload{0x00, 0x01, 0, 0};
        payload.insert(payload.end(), 256, 'n');
        payload.push_back(0xff);
        auto file = parseRawFile(payload, filename);
        REQUIRE(file.has_value());
        REQUIRE(*file == std::vector<uint8_t>{0xff});
        REQUIRE(filename == std::string(256, 'n'));
    }

    SECTION("an empty file") {
        auto file = parseRawFile({1, 0, 0, 0, 'x'}, filename);
        REQUIRE(file.has_value());
        REQUIRE(file->empty());
        REQUIRE(filename == "x");
    }

    SECTION("an empty filename") {
        auto file = parseRawFile({0, 0, 0, 0, 9, 8}, filename);
        REQUIRE(file.has_value());
        REQUIRE(*file == std::vector<uint8_t>{9, 8});
        REQUIRE(filename.empty());
    }

    SECTION("bytes that look like text stay bytes") {
        auto file = parseRawFile({1, 0, 0, 0, 'x', 0, '"', '\n', 0xff}, filename);
        REQUIRE(file.has_value());
        REQUIRE(*file == std::vector<uint8_t>{0, '"', '\n', 0xff});
    }
}

TEST_CASE("parseRawFile rejects a payload whose lengths do not fit") {
    std::string filename = "unchanged";

    SECTION("no length field") {
        REQUIRE_FALSE(parseRawFile({}, filename).has_value());
        REQUIRE_FALSE(parseRawFile({1, 0, 0}, filename).has_value());
    }

    SECTION("filename longer than the payload") {
        REQUIRE_FALSE(parseRawFile({4, 0, 0, 0, 'a', 'b', 'c'}, filename).has_value());
        REQUIRE_FALSE(parseRawFile({1, 0, 0, 0}, filename).has_value());
    }

    SECTION("a length that overflows a 32 bit sum") {
        REQUIRE_FALSE(parseRawFile({0xff, 0xff, 0xff, 0xff, 'a'}, filename).has_value());
    }

    REQUIRE(filename == "unchanged");
}
