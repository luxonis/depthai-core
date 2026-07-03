#include <catch2/catch_all.hpp>

#include <chrono>
#include <cstdint>
#include <filesystem>
#include <optional>
#include <string>
#include <variant>
#include <vector>

#include "depthai/pipeline/datatype/EncodedFrame.hpp"
#include "depthai/pipeline/datatype/IMUData.hpp"
#include "depthai/pipeline/datatype/ImgFrame.hpp"
#include "depthai/pipeline/datatype/PointCloudData.hpp"
#include "depthai/pipeline/datatype/RGBDData.hpp"

namespace fs = std::filesystem;

namespace {

class TempFile {
   public:
    explicit TempFile(const std::string& stem)
        : path(fs::temp_directory_path() / (stem + "_" + std::to_string(std::chrono::steady_clock::now().time_since_epoch().count()) + ".pb")) {}

    ~TempFile() {
        std::error_code ec;
        fs::remove(path, ec);
    }

    fs::path path;
};

std::vector<uint8_t> toVector(dai::span<const uint8_t> data) {
    return std::vector<uint8_t>(data.begin(), data.end());
}

}  // namespace

#ifndef DEPTHAI_ENABLE_PROTOBUF

TEST_CASE("Proto file I/O requires protobuf support", "[ProtoFileIO]") {
    SUCCEED("Protobuf support disabled");
}

#else

TEST_CASE("ImgFrame save/load roundtrip", "[ProtoFileIO][ImgFrame]") {
    dai::ImgFrame frame;
    frame.setInstanceNum(4).setCategory(9).setSize(8, 6).setSourceSize(8, 6).setType(dai::ImgFrame::Type::RAW8);
    frame.setSequenceNum(123);
    frame.cam.exposureTimeUs = 2500;
    frame.setData(std::vector<uint8_t>{1, 2, 3, 4, 5, 6, 7, 8});

    TempFile file("imgframe_roundtrip");
    frame.save(file.path);

    dai::ImgFrame restored;
    restored.load(file.path);

    REQUIRE(restored.getInstanceNum() == frame.getInstanceNum());
    REQUIRE(restored.getCategory() == frame.getCategory());
    REQUIRE(restored.getWidth() == frame.getWidth());
    REQUIRE(restored.getHeight() == frame.getHeight());
    REQUIRE(restored.getType() == frame.getType());
    REQUIRE(restored.getSequenceNum() == frame.getSequenceNum());
    REQUIRE(restored.getExposureTime() == frame.getExposureTime());
    REQUIRE(toVector(restored.getData()) == toVector(frame.getData()));
}

TEST_CASE("ImgFrame metadata-only save/load clears payload", "[ProtoFileIO][ImgFrame]") {
    dai::ImgFrame frame;
    frame.setSize(4, 4).setSourceSize(4, 4).setType(dai::ImgFrame::Type::RAW8);
    frame.setData(std::vector<uint8_t>{9, 8, 7, 6});

    TempFile file("imgframe_metadata_only");
    frame.save(file.path, true);

    dai::ImgFrame restored;
    restored.load(file.path, true);

    REQUIRE(restored.getWidth() == frame.getWidth());
    REQUIRE(restored.getHeight() == frame.getHeight());
    REQUIRE(restored.getType() == frame.getType());
    REQUIRE(restored.getData().empty());
}

TEST_CASE("EncodedFrame save/load roundtrip", "[ProtoFileIO][EncodedFrame]") {
    dai::EncodedFrame frame;
    frame.setInstanceNum(3).setSize(320, 180).setQuality(91).setBitrate(1234567).setProfile(dai::EncodedFrame::Profile::JPEG).setFrameType(
        dai::EncodedFrame::FrameType::I);
    frame.setLossless(false);
    frame.setSequenceNum(777);
    frame.setData(std::vector<uint8_t>{10, 20, 30, 40, 50});

    TempFile file("encodedframe_roundtrip");
    frame.save(file.path);

    dai::EncodedFrame restored;
    restored.load(file.path);

    REQUIRE(restored.getInstanceNum() == frame.getInstanceNum());
    REQUIRE(restored.getWidth() == frame.getWidth());
    REQUIRE(restored.getHeight() == frame.getHeight());
    REQUIRE(restored.getQuality() == frame.getQuality());
    REQUIRE(restored.getBitrate() == frame.getBitrate());
    REQUIRE(restored.getProfile() == frame.getProfile());
    REQUIRE(restored.getFrameType() == frame.getFrameType());
    REQUIRE(restored.getLossless() == frame.getLossless());
    REQUIRE(restored.getSequenceNum() == frame.getSequenceNum());
    REQUIRE(toVector(restored.getData()) == toVector(frame.getData()));
}

TEST_CASE("IMUData save/load roundtrip", "[ProtoFileIO][IMUData]") {
    dai::IMUData imu;
    imu.setSequenceNum(42);
    dai::IMUPacket packet;
    packet.acceleroMeter.sequence = 100;
    packet.acceleroMeter.x = 1.5F;
    packet.acceleroMeter.y = -2.0F;
    packet.acceleroMeter.z = 9.81F;
    packet.gyroscope.sequence = 101;
    packet.gyroscope.x = 0.1F;
    packet.gyroscope.y = 0.2F;
    packet.gyroscope.z = 0.3F;
    imu.packets.push_back(packet);

    TempFile file("imudata_roundtrip");
    imu.save(file.path);

    dai::IMUData restored;
    restored.load(file.path);

    REQUIRE(restored.getSequenceNum() == imu.getSequenceNum());
    REQUIRE(restored.packets.size() == 1);
    REQUIRE(restored.packets[0].acceleroMeter.sequence == 100);
    REQUIRE(restored.packets[0].acceleroMeter.x == Catch::Approx(1.5F));
    REQUIRE(restored.packets[0].acceleroMeter.z == Catch::Approx(9.81F));
    REQUIRE(restored.packets[0].gyroscope.sequence == 101);
    REQUIRE(restored.packets[0].gyroscope.y == Catch::Approx(0.2F));
}

TEST_CASE("PointCloudData save/load roundtrip", "[ProtoFileIO][PointCloudData]") {
    dai::PointCloudData pcd;
    pcd.setWidth(2).setHeight(2).setInstanceNum(6).setPoints({{1.F, 2.F, 3.F}, {4.F, 5.F, 6.F}, {7.F, 8.F, 9.F}, {10.F, 11.F, 12.F}});
    pcd.updateBoundingBox();
    pcd.setSequenceNum(55);

    TempFile file("pointcloud_roundtrip");
    pcd.save(file.path);

    dai::PointCloudData restored;
    restored.load(file.path);

    REQUIRE(restored.getWidth() == pcd.getWidth());
    REQUIRE(restored.getHeight() == pcd.getHeight());
    REQUIRE(restored.getInstanceNum() == pcd.getInstanceNum());
    REQUIRE(restored.getSequenceNum() == pcd.getSequenceNum());
    REQUIRE(restored.getMinX() == Catch::Approx(pcd.getMinX()));
    REQUIRE(restored.getMaxZ() == Catch::Approx(pcd.getMaxZ()));
    REQUIRE(restored.getPoints().size() == pcd.getPoints().size());
    REQUIRE(restored.getPoints()[2].z == Catch::Approx(9.F));
}

TEST_CASE("RGBDData save/load roundtrip", "[ProtoFileIO][RGBDData]") {
    auto color = std::make_shared<dai::ImgFrame>();
    color->setSize(4, 3).setSourceSize(4, 3).setType(dai::ImgFrame::Type::RAW8).setInstanceNum(1);
    color->setData(std::vector<uint8_t>{1, 2, 3, 4});

    auto depth = std::make_shared<dai::EncodedFrame>();
    depth->setSize(4, 3).setProfile(dai::EncodedFrame::Profile::JPEG).setFrameType(dai::EncodedFrame::FrameType::I).setInstanceNum(2);
    depth->setData(std::vector<uint8_t>{9, 8, 7});

    dai::RGBDData rgbd;
    rgbd.setSequenceNum(999);
    rgbd.setRGBFrame(color);
    rgbd.setDepthFrame(depth);

    TempFile file("rgbd_roundtrip");
    rgbd.save(file.path);

    dai::RGBDData restored;
    restored.load(file.path);

    REQUIRE(restored.getSequenceNum() == rgbd.getSequenceNum());

    auto rgb = restored.getRGBFrame();
    REQUIRE(rgb.has_value());
    REQUIRE(std::holds_alternative<std::shared_ptr<dai::ImgFrame>>(*rgb));
    auto restoredColor = std::get<std::shared_ptr<dai::ImgFrame>>(*rgb);
    REQUIRE(restoredColor != nullptr);
    REQUIRE(restoredColor->getWidth() == color->getWidth());
    REQUIRE(restoredColor->getType() == color->getType());
    REQUIRE(toVector(restoredColor->getData()) == toVector(color->getData()));

    auto depthFrame = restored.getDepthFrame();
    REQUIRE(depthFrame.has_value());
    REQUIRE(std::holds_alternative<std::shared_ptr<dai::EncodedFrame>>(*depthFrame));
    auto restoredDepth = std::get<std::shared_ptr<dai::EncodedFrame>>(*depthFrame);
    REQUIRE(restoredDepth != nullptr);
    REQUIRE(restoredDepth->getWidth() == depth->getWidth());
    REQUIRE(restoredDepth->getProfile() == depth->getProfile());
    REQUIRE(toVector(restoredDepth->getData()) == toVector(depth->getData()));
}

#endif
