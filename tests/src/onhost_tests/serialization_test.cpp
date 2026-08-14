#include <catch2/catch_all.hpp>

// Include depthai library
#include <depthai/depthai.hpp>

namespace {
/// Extrinsics as devices whose firmware predates the device-qualified reference frames know it
struct LegacyExtrinsics {
    std::vector<std::vector<float>> rotationMatrix;
    dai::Point3f translation;
    dai::Point3f specTranslation;
    dai::CameraBoardSocket toCameraSocket = dai::CameraBoardSocket::AUTO;
    dai::LengthUnit lengthUnit = dai::LengthUnit::CENTIMETER;

    DEPTHAI_SERIALIZE(LegacyExtrinsics, rotationMatrix, translation, specTranslation, toCameraSocket, lengthUnit);
};
}  // namespace

TEST_CASE("Extrinsics keep the wire format of devices that do not know the device id") {
    dai::Extrinsics extrinsics;
    extrinsics.translation = dai::Point3f(1.0F, 2.0F, 3.0F);
    extrinsics.setReferenceFrame({"14442C10D1", dai::CameraBoardSocket::CAM_B});

    SECTION("what the host sends can be read by a device") {
        LegacyExtrinsics legacy;
        REQUIRE_NOTHROW(dai::utility::deserialize(dai::utility::serialize(extrinsics), legacy));
        REQUIRE(legacy.translation.z == 3.0F);
        REQUIRE(legacy.toCameraSocket == dai::CameraBoardSocket::CAM_B);
    }

    SECTION("what a device sends can be read by the host, with an unknown device") {
        LegacyExtrinsics legacy;
        legacy.translation = dai::Point3f(1.0F, 2.0F, 3.0F);
        legacy.toCameraSocket = dai::CameraBoardSocket::CAM_B;

        dai::Extrinsics deserialized;
        REQUIRE_NOTHROW(dai::utility::deserialize(dai::utility::serialize(legacy), deserialized));
        REQUIRE(deserialized.translation.z == 3.0F);
        REQUIRE(deserialized.getReferenceFrame() == dai::CoordinateFrame{"", dai::CameraBoardSocket::CAM_B});
    }

    SECTION("the device id is kept in json") {
        const nlohmann::json json = extrinsics;
        REQUIRE(json["toDeviceId"] == "14442C10D1");
        REQUIRE(json.get<dai::Extrinsics>().toDeviceId == std::make_optional<std::string>("14442C10D1"));
    }
}

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
