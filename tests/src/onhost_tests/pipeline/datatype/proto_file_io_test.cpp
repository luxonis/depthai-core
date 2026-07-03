#include <catch2/catch_all.hpp>

#include <chrono>
#include <cstdint>
#include <filesystem>
#include <fstream>
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
        : path(fs::temp_directory_path() / (stem + "_" + std::to_string(std::chrono::steady_clock::now().time_since_epoch().count()))) {}

    ~TempFile() {
        std::error_code ec;
        fs::remove(path, ec);
        fs::remove(path.string() + ".dai", ec);
    }

    fs::path path;
};

fs::path withDaiExtension(const fs::path& path) {
    auto resolved = path;
    resolved += ".dai";
    return resolved;
}

fs::path withCustomExtension(const fs::path& path, const std::string& extension) {
    auto resolved = path;
    resolved += extension;
    return resolved;
}

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

    SECTION("Extensionless path resolves to .dai for save and load") {
        TempFile file("imgframe_roundtrip_no_ext");
        frame.save(file.path);
        REQUIRE(fs::exists(withDaiExtension(file.path)));

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

    SECTION("Explicit .dai path works the same for save and load") {
        TempFile file("imgframe_roundtrip_ext");
        auto explicitPath = withDaiExtension(file.path);
        frame.save(explicitPath);
        REQUIRE(fs::exists(explicitPath));

        dai::ImgFrame restored;
        restored.load(explicitPath);

        REQUIRE(restored.getInstanceNum() == frame.getInstanceNum());
        REQUIRE(restored.getCategory() == frame.getCategory());
        REQUIRE(restored.getWidth() == frame.getWidth());
        REQUIRE(restored.getHeight() == frame.getHeight());
        REQUIRE(restored.getType() == frame.getType());
        REQUIRE(restored.getSequenceNum() == frame.getSequenceNum());
        REQUIRE(restored.getExposureTime() == frame.getExposureTime());
        REQUIRE(toVector(restored.getData()) == toVector(frame.getData()));
    }
}

TEST_CASE("ImgFrame metadata-only save/load clears payload", "[ProtoFileIO][ImgFrame]") {
    dai::ImgFrame frame;
    frame.setSize(4, 4).setSourceSize(4, 4).setType(dai::ImgFrame::Type::RAW8);
    frame.setData(std::vector<uint8_t>{9, 8, 7, 6});

    SECTION("Extensionless metadata-only path resolves to .dai") {
        TempFile file("imgframe_metadata_only_no_ext");
        frame.save(file.path, true);
        REQUIRE(fs::exists(withDaiExtension(file.path)));

        dai::ImgFrame restored;
        restored.load(file.path, true);

        REQUIRE(restored.getWidth() == frame.getWidth());
        REQUIRE(restored.getHeight() == frame.getHeight());
        REQUIRE(restored.getType() == frame.getType());
        REQUIRE(restored.getData().empty());
    }

    SECTION("Explicit .dai metadata-only path works the same") {
        TempFile file("imgframe_metadata_only_ext");
        auto explicitPath = withDaiExtension(file.path);
        frame.save(explicitPath, true);
        REQUIRE(fs::exists(explicitPath));

        dai::ImgFrame restored;
        restored.load(explicitPath, true);

        REQUIRE(restored.getWidth() == frame.getWidth());
        REQUIRE(restored.getHeight() == frame.getHeight());
        REQUIRE(restored.getType() == frame.getType());
        REQUIRE(restored.getData().empty());
    }
}

TEST_CASE("EncodedFrame save/load roundtrip", "[ProtoFileIO][EncodedFrame]") {
    dai::EncodedFrame frame;
    frame.setInstanceNum(3).setSize(320, 180).setQuality(91).setBitrate(1234567).setProfile(dai::EncodedFrame::Profile::JPEG).setFrameType(
        dai::EncodedFrame::FrameType::I);
    frame.setLossless(false);
    frame.setSequenceNum(777);
    frame.setData(std::vector<uint8_t>{10, 20, 30, 40, 50});

    SECTION("Extensionless path resolves to .dai for save and load") {
        TempFile file("encodedframe_roundtrip_no_ext");
        frame.save(file.path);
        REQUIRE(fs::exists(withDaiExtension(file.path)));

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

    SECTION("Explicit .dai path works the same for save and load") {
        TempFile file("encodedframe_roundtrip_ext");
        auto explicitPath = withDaiExtension(file.path);
        frame.save(explicitPath);
        REQUIRE(fs::exists(explicitPath));

        dai::EncodedFrame restored;
        restored.load(explicitPath);

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

    SECTION("Explicit non-.dai extension is preserved") {
        TempFile file("encodedframe_roundtrip_bin");
        auto explicitPath = withCustomExtension(file.path, ".bin");
        frame.save(explicitPath);
        REQUIRE(fs::exists(explicitPath));
        REQUIRE_FALSE(fs::exists(withDaiExtension(explicitPath)));

        dai::EncodedFrame restored;
        restored.load(explicitPath);

        REQUIRE(restored.getWidth() == frame.getWidth());
        REQUIRE(restored.getProfile() == frame.getProfile());
        REQUIRE(toVector(restored.getData()) == toVector(frame.getData()));
    }
}

TEST_CASE("EncodedFrame metadata-only save/load clears payload", "[ProtoFileIO][EncodedFrame]") {
    dai::EncodedFrame frame;
    frame.setInstanceNum(5).setSize(128, 72).setProfile(dai::EncodedFrame::Profile::JPEG).setFrameType(dai::EncodedFrame::FrameType::I);
    frame.setQuality(80);
    frame.setData(std::vector<uint8_t>{5, 4, 3, 2, 1});

    SECTION("Extensionless metadata-only path resolves to .dai") {
        TempFile file("encodedframe_metadata_only_no_ext");
        frame.save(file.path, true);
        REQUIRE(fs::exists(withDaiExtension(file.path)));

        dai::EncodedFrame restored;
        restored.load(file.path, true);

        REQUIRE(restored.getInstanceNum() == frame.getInstanceNum());
        REQUIRE(restored.getWidth() == frame.getWidth());
        REQUIRE(restored.getHeight() == frame.getHeight());
        REQUIRE(restored.getProfile() == frame.getProfile());
        REQUIRE(restored.getFrameType() == frame.getFrameType());
        REQUIRE(restored.getData().empty());
    }

    SECTION("Explicit .dai metadata-only path works the same") {
        TempFile file("encodedframe_metadata_only_ext");
        auto explicitPath = withDaiExtension(file.path);
        frame.save(explicitPath, true);
        REQUIRE(fs::exists(explicitPath));

        dai::EncodedFrame restored;
        restored.load(explicitPath, true);

        REQUIRE(restored.getInstanceNum() == frame.getInstanceNum());
        REQUIRE(restored.getWidth() == frame.getWidth());
        REQUIRE(restored.getHeight() == frame.getHeight());
        REQUIRE(restored.getProfile() == frame.getProfile());
        REQUIRE(restored.getFrameType() == frame.getFrameType());
        REQUIRE(restored.getData().empty());
    }
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

    SECTION("Extensionless path resolves to .dai for save and load") {
        TempFile file("imudata_roundtrip_no_ext");
        imu.save(file.path);
        REQUIRE(fs::exists(withDaiExtension(file.path)));

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

    SECTION("Explicit .dai path works the same for save and load") {
        TempFile file("imudata_roundtrip_ext");
        auto explicitPath = withDaiExtension(file.path);
        imu.save(explicitPath);
        REQUIRE(fs::exists(explicitPath));

        dai::IMUData restored;
        restored.load(explicitPath);

        REQUIRE(restored.getSequenceNum() == imu.getSequenceNum());
        REQUIRE(restored.packets.size() == 1);
        REQUIRE(restored.packets[0].acceleroMeter.sequence == 100);
        REQUIRE(restored.packets[0].acceleroMeter.x == Catch::Approx(1.5F));
        REQUIRE(restored.packets[0].acceleroMeter.z == Catch::Approx(9.81F));
        REQUIRE(restored.packets[0].gyroscope.sequence == 101);
        REQUIRE(restored.packets[0].gyroscope.y == Catch::Approx(0.2F));
    }
}

TEST_CASE("PointCloudData save/load roundtrip", "[ProtoFileIO][PointCloudData]") {
    dai::PointCloudData pcd;
    pcd.setWidth(2).setHeight(2).setInstanceNum(6).setPoints({{1.F, 2.F, 3.F}, {4.F, 5.F, 6.F}, {7.F, 8.F, 9.F}, {10.F, 11.F, 12.F}});
    pcd.updateBoundingBox();
    pcd.setSequenceNum(55);

    SECTION("Extensionless path resolves to .dai for save and load") {
        TempFile file("pointcloud_roundtrip_no_ext");
        pcd.save(file.path);
        REQUIRE(fs::exists(withDaiExtension(file.path)));

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

    SECTION("Explicit .dai path works the same for save and load") {
        TempFile file("pointcloud_roundtrip_ext");
        auto explicitPath = withDaiExtension(file.path);
        pcd.save(explicitPath);
        REQUIRE(fs::exists(explicitPath));

        dai::PointCloudData restored;
        restored.load(explicitPath);

        REQUIRE(restored.getWidth() == pcd.getWidth());
        REQUIRE(restored.getHeight() == pcd.getHeight());
        REQUIRE(restored.getInstanceNum() == pcd.getInstanceNum());
        REQUIRE(restored.getSequenceNum() == pcd.getSequenceNum());
        REQUIRE(restored.getMinX() == Catch::Approx(pcd.getMinX()));
        REQUIRE(restored.getMaxZ() == Catch::Approx(pcd.getMaxZ()));
        REQUIRE(restored.getPoints().size() == pcd.getPoints().size());
        REQUIRE(restored.getPoints()[2].z == Catch::Approx(9.F));
    }
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

    SECTION("Extensionless path resolves to .dai for save and load") {
        TempFile file("rgbd_roundtrip_no_ext");
        rgbd.save(file.path);
        REQUIRE(fs::exists(withDaiExtension(file.path)));

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

    SECTION("Explicit .dai path works the same for save and load") {
        TempFile file("rgbd_roundtrip_ext");
        auto explicitPath = withDaiExtension(file.path);
        rgbd.save(explicitPath);
        REQUIRE(fs::exists(explicitPath));

        dai::RGBDData restored;
        restored.load(explicitPath);

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
}

TEST_CASE("Proto file I/O reports missing and invalid files", "[ProtoFileIO][Failures]") {
    SECTION("Missing file throws for extensionless path") {
        TempFile file("missing_proto_file");
        dai::ImgFrame frame;
        REQUIRE_THROWS_AS(frame.load(file.path), std::runtime_error);
    }

    SECTION("Missing file throws for explicit .dai path") {
        TempFile file("missing_proto_file_dai");
        auto explicitPath = withDaiExtension(file.path);
        dai::EncodedFrame frame;
        REQUIRE_THROWS_AS(frame.load(explicitPath), std::runtime_error);
    }

    SECTION("Invalid protobuf payload throws") {
        TempFile file("invalid_proto_file");
        auto invalidPath = withDaiExtension(file.path);
        {
            std::ofstream out(invalidPath, std::ios::binary);
            REQUIRE(out.good());
            out << "not a protobuf payload";
        }

        dai::IMUData imu;
        REQUIRE_THROWS_AS(imu.load(invalidPath), std::runtime_error);
    }
}

#endif
