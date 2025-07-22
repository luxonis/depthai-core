#include "depthai/pipeline/node/host/Replay.hpp"

#include <catch2/catch_all.hpp>
#include <catch2/catch_test_macros.hpp>
#include <cstdlib>
#include <ctime>
#include <exception>
#include <filesystem>

#include "../../src/utility/Platform.hpp"
#include "depthai/depthai.hpp"
#include "depthai/pipeline/node/host/Record.hpp"
#include "depthai/utility/Compression.hpp"

constexpr unsigned int NUM_MSGS = 100;

class TestHelper {
   public:
    TestHelper() {
        testFolder = dai::platform::getTempPath();
        std::filesystem::create_directories(testFolder);
        std::filesystem::create_directories(std::filesystem::path(testFolder).append("extracted"));

        auto recordingFilenames = dai::utility::filenamesInTar(RECORDING_PATH);
        std::vector<std::filesystem::path> recordingExtFiles;
        recordingExtFiles.reserve(recordingFilenames.size());
        for(const auto& filename : recordingFilenames) {
            recordingExtFiles.push_back(std::filesystem::path(testFolder).append("extracted").append(filename));
        }
        dai::utility::untarFiles(RECORDING_PATH, recordingFilenames, recordingExtFiles);
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
