#include "depthai/pipeline/node/host/Replay.hpp"

#include <catch2/catch_all.hpp>
#include <catch2/catch_test_macros.hpp>
#include <chrono>
#include <cstdlib>
#include <ctime>
#include <exception>
#include <filesystem>
#include <variant>

#include "../../src/utility/Platform.hpp"
#include "../../src/utility/RecordReplayImpl.hpp"
#include "depthai/depthai.hpp"
#include "depthai/pipeline/node/host/Record.hpp"
#include "depthai/utility/Compression.hpp"
#ifdef DEPTHAI_ENABLE_PROTOBUF
    #include "depthai/schemas/RGBDData.pb.h"
#endif

constexpr unsigned int NUM_MSGS = 100;

// Disable container overflow detection for this test binary (false positive from protobuf)
extern "C" const char* __asan_default_options() {
    return "detect_container_overflow=0";
}

namespace {

std::vector<std::uint8_t> toVector(dai::span<const std::uint8_t> data) {
    return {data.begin(), data.end()};
}

}  // namespace

class TestHelper {
   public:
    TestHelper() {
        testFolder = dai::platform::getTempPath();
        std::filesystem::create_directories(testFolder);
        std::filesystem::create_directories(std::filesystem::path(testFolder).append("extracted"));

        auto recordingFilenames = dai::utility::filenamesInArchive(RECORDING_PATH);
        std::vector<std::filesystem::path> recordingExtFiles;
        recordingExtFiles.reserve(recordingFilenames.size());
        for(const auto& filename : recordingFilenames) {
            recordingExtFiles.push_back(std::filesystem::path(testFolder).append("extracted").append(filename));
        }
        dai::utility::extractFiles(RECORDING_PATH, recordingFilenames, recordingExtFiles);
    }

    ~TestHelper() {
        try {
            std::filesystem::remove_all(testFolder);
        } catch(const std::exception& e) {
            std::cerr << "Failed to remove test folder: " << e.what() << std::endl;
        }
    }

    std::filesystem::path testFolder;
};

TEST_CASE("ReplayMetadataOnly node") {
    {
        TestHelper helper;

        dai::Pipeline p(false);

        auto replayNode = p.create<dai::node::ReplayMetadataOnly>();
        replayNode->setReplayFile(std::filesystem::path(helper.testFolder).append("extracted").append("IMU.mcap"));
        replayNode->setLoop(true);

        auto q = replayNode->out.createOutputQueue();

        p.start();
        for(auto i = 0U; i < NUM_MSGS; i++) {
            if(!p.isRunning()) break;
            auto data = q->get<dai::IMUData>();
            REQUIRE(data != nullptr);
        }
        p.stop();
    }
}

#ifdef DEPTHAI_ENABLE_PROTOBUF
TEST_CASE("ReplayMetadataOnly replays RGBDData MCAP", "[Replay][RGBDData]") {
    TestHelper helper;
    const auto replayPath = std::filesystem::path(helper.testFolder).append("rgbd.mcap");

    auto colorFrame = std::make_shared<dai::ImgFrame>();
    colorFrame->setSequenceNum(1);
    colorFrame->setTimestamp(std::chrono::steady_clock::time_point(std::chrono::milliseconds(10)));
    colorFrame->setTimestampDevice(std::chrono::steady_clock::time_point(std::chrono::milliseconds(20)));
    colorFrame->setTimestampSystem(std::chrono::system_clock::time_point(std::chrono::milliseconds(30)));
    colorFrame->setInstanceNum(2).setCategory(3).setType(dai::ImgFrame::Type::BGR888i);
    colorFrame->setSourceSize(2, 1);
    colorFrame->setSize(2, 1);
    colorFrame->setData(std::vector<std::uint8_t>{1, 2, 3, 4, 5, 6});

    auto depthFrame = std::make_shared<dai::EncodedFrame>();
    depthFrame->setSequenceNum(4);
    depthFrame->setTimestamp(std::chrono::steady_clock::time_point(std::chrono::milliseconds(40)));
    depthFrame->setTimestampDevice(std::chrono::steady_clock::time_point(std::chrono::milliseconds(50)));
    depthFrame->setTimestampSystem(std::chrono::system_clock::time_point(std::chrono::milliseconds(60)));
    depthFrame->setInstanceNum(5).setSize(2, 1).setQuality(90).setBitrate(1024).setLossless(false);
    depthFrame->setProfile(dai::EncodedFrame::Profile::AVC).setFrameType(dai::EncodedFrame::FrameType::P);
    depthFrame->frameOffset = 7;
    depthFrame->frameSize = 3;
    depthFrame->setData(std::vector<std::uint8_t>{7, 8, 9});

    dai::RGBDData source;
    source.setSequenceNum(8);
    source.setTimestamp(std::chrono::steady_clock::time_point(std::chrono::milliseconds(80)));
    source.setTimestampDevice(std::chrono::steady_clock::time_point(std::chrono::milliseconds(90)));
    source.setTimestampSystem(std::chrono::system_clock::time_point(std::chrono::milliseconds(100)));
    source.setRGBFrame(colorFrame);
    source.setDepthFrame(depthFrame);

    dai::utility::ByteRecorder recorder;
    recorder.init<dai::proto::rgbd_data::RGBDData>(replayPath.string(), dai::RecordConfig::CompressionLevel::NONE, "rgbd");
    recorder.write(source.serializeProto(false));
    recorder.close();

    dai::Pipeline p(false);
    auto replayNode = p.create<dai::node::ReplayMetadataOnly>();
    replayNode->setReplayFile(replayPath);
    replayNode->setLoop(true);
    auto q = replayNode->out.createOutputQueue();

    p.start();
    REQUIRE(q->get<dai::RGBDData>() != nullptr);
    REQUIRE(q->get<dai::RGBDData>() != nullptr);
    auto data = q->get<dai::RGBDData>();
    REQUIRE(data != nullptr);
    const auto sequenceNumOffset = data->getSequenceNum() - source.getSequenceNum();
    const auto timestampOffset = data->getTimestamp() - source.getTimestamp();
    const auto deviceTimestampOffset = data->getTimestampDevice() - source.getTimestampDevice();
    REQUIRE(sequenceNumOffset != 0);
    REQUIRE(timestampOffset != std::chrono::steady_clock::duration::zero());
    REQUIRE(deviceTimestampOffset != std::chrono::steady_clock::duration::zero());
    REQUIRE(data->getRGBFrame().has_value());
    REQUIRE(data->getDepthFrame().has_value());
    REQUIRE(std::holds_alternative<std::shared_ptr<dai::ImgFrame>>(data->getRGBFrame().value()));
    REQUIRE(std::holds_alternative<std::shared_ptr<dai::EncodedFrame>>(data->getDepthFrame().value()));

    const auto replayedColor = std::get<std::shared_ptr<dai::ImgFrame>>(data->getRGBFrame().value());
    const auto replayedDepth = std::get<std::shared_ptr<dai::EncodedFrame>>(data->getDepthFrame().value());
    REQUIRE(replayedColor != nullptr);
    REQUIRE(replayedDepth != nullptr);
    REQUIRE(replayedColor->getWidth() == colorFrame->getWidth());
    REQUIRE(replayedColor->getHeight() == colorFrame->getHeight());
    REQUIRE(toVector(replayedColor->getData()) == toVector(colorFrame->getData()));
    REQUIRE(replayedColor->getSequenceNum() == colorFrame->getSequenceNum() + sequenceNumOffset);
    REQUIRE(replayedColor->getTimestamp() == colorFrame->getTimestamp() + timestampOffset);
    REQUIRE(replayedColor->getTimestampDevice() == colorFrame->getTimestampDevice() + deviceTimestampOffset);
    REQUIRE(replayedColor->getTimestampSystem() == colorFrame->getTimestampSystem());
    REQUIRE(replayedDepth->getWidth() == depthFrame->getWidth());
    REQUIRE(replayedDepth->getHeight() == depthFrame->getHeight());
    REQUIRE(toVector(replayedDepth->getData()) == toVector(depthFrame->getData()));
    REQUIRE(replayedDepth->getSequenceNum() == depthFrame->getSequenceNum() + sequenceNumOffset);
    REQUIRE(replayedDepth->getTimestamp() == depthFrame->getTimestamp() + timestampOffset);
    REQUIRE(replayedDepth->getTimestampDevice() == depthFrame->getTimestampDevice() + deviceTimestampOffset);
    REQUIRE(replayedDepth->getTimestampSystem() == depthFrame->getTimestampSystem());
    p.stop();
}
#endif

TEST_CASE("ReplayVideo node") {
    {
        TestHelper helper;

        dai::Pipeline p(false);

        auto replayNode = p.create<dai::node::ReplayVideo>();
        replayNode->setReplayMetadataFile(std::filesystem::path(helper.testFolder).append("extracted").append("CameraCAM_A.mcap"));
        replayNode->setReplayVideoFile(std::filesystem::path(helper.testFolder).append("extracted").append("CameraCAM_A.mp4"));
        replayNode->setLoop(true);

        auto q = replayNode->out.createOutputQueue();

        p.start();
        for(auto i = 0U; i < NUM_MSGS; i++) {
            if(!p.isRunning()) break;
            auto data = q->get<dai::ImgFrame>();
            REQUIRE(data != nullptr);
        }
        p.stop();
    }
}

TEST_CASE("ReplayVideo no metadata") {
    {
        TestHelper helper;

        dai::Pipeline p(false);

        auto replayNode = p.create<dai::node::ReplayVideo>();
        replayNode->setReplayVideoFile(std::filesystem::path(helper.testFolder).append("extracted").append("CameraCAM_A.mp4"));
        replayNode->setOutFrameType(dai::ImgFrame::Type::NV12);
        replayNode->setLoop(true);

        auto q = replayNode->out.createOutputQueue();

        p.start();
        for(auto i = 0U; i < NUM_MSGS; i++) {
            if(!p.isRunning()) break;
            auto data = q->get<dai::ImgFrame>();
            REQUIRE(data != nullptr);
            REQUIRE(data->getWidth() > 0);
            REQUIRE(data->getHeight() > 0);
            REQUIRE(data->getType() == dai::ImgFrame::Type::NV12);
            REQUIRE(data->validateTransformations());
        }
        p.stop();
    }
}
