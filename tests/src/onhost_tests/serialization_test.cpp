#include <catch2/catch_all.hpp>

// Include depthai library
#include <depthai/depthai.hpp>

TEST_CASE("Roundtrip") {
    dai::Pipeline p;
    auto stereo = p.create<dai::node::StereoDepth>();

    // Create ground truth properties
    stereo->setInputResolution(0xa0a0a0a0, 0xa0a0a0a0);
    stereo->setOutputSize(0x55555555, 0x55555555);
    stereo->setExtendedDisparity(true);
    stereo->properties.numFramesPool = 42;

    // Round trip
    {
        auto ser = dai::utility::serialize(stereo->properties);
        dai::node::StereoDepth::Properties des;
        dai::utility::deserialize(ser, des);
        for(uint8_t b : ser) {
            printf("%02X ", b);
        }
        printf("\n");

        REQUIRE(des.width.value() == (int)0xa0a0a0a0);
        REQUIRE(des.height.value() == (int)0xa0a0a0a0);
        REQUIRE(des.outWidth.value() == (int)0x55555555);
        REQUIRE(des.outHeight.value() == (int)0x55555555);
        REQUIRE(des.numFramesPool == 42);
    }

    // Round trip through pipeline
    {
        auto ser = p.getPipelineSchema().nodes[0].properties;
        dai::node::StereoDepth::Properties des;
        dai::utility::deserialize(ser, des);
        for(uint8_t b : ser) {
            printf("%02X ", b);
        }
        printf("\n");

        REQUIRE(des.width.value() == (int)0xa0a0a0a0);
        REQUIRE(des.height.value() == (int)0xa0a0a0a0);
        REQUIRE(des.outWidth.value() == (int)0x55555555);
        REQUIRE(des.outHeight.value() == (int)0x55555555);
        REQUIRE(des.numFramesPool == 42);
    }
}

TEST_CASE("EEPROM multi-device poses round-trip through every supported serialization format") {
    dai::EepromData eeprom;
    dai::Extrinsics extrinsics;
    extrinsics.rotationMatrix = {{1.0f, 0.0f, 0.0f}, {0.0f, 1.0f, 0.0f}, {0.0f, 0.0f, 1.0f}};
    extrinsics.translation = {10.0f, -2.5f, 3.0f};
    extrinsics.specTranslation = {11.0f, -3.0f, 4.0f};
    extrinsics.toCameraSocket = dai::CameraBoardSocket::CAM_B;
    eeprom.devicesData["device-a"][dai::CameraBoardSocket::CAM_A] = extrinsics;

    const auto requireRoundTrip = [&eeprom, &extrinsics](dai::SerializationType format) {
        dai::EepromData restored;
        const auto serialized = dai::utility::serialize(eeprom, format);
        REQUIRE_FALSE(serialized.empty());
        REQUIRE(dai::utility::deserialize(serialized, restored, format));

        const auto& pose = restored.devicesData.at("device-a").at(dai::CameraBoardSocket::CAM_A);
        REQUIRE(pose.toCameraSocket == dai::CameraBoardSocket::CAM_B);
        REQUIRE(pose.translation.x == Catch::Approx(10.0f));
        REQUIRE(pose.translation.y == Catch::Approx(-2.5f));
        REQUIRE(pose.translation.z == Catch::Approx(3.0f));
        REQUIRE(pose.specTranslation.x == Catch::Approx(11.0f));
        REQUIRE(pose.rotationMatrix == extrinsics.rotationMatrix);
    };

    SECTION("libnop") {
        requireRoundTrip(dai::SerializationType::LIBNOP);
    }
    SECTION("JSON") {
        requireRoundTrip(dai::SerializationType::JSON);
    }
    SECTION("MessagePack JSON") {
        requireRoundTrip(dai::SerializationType::JSON_MSGPACK);
    }
}
