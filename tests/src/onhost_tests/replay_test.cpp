#include "depthai/pipeline/node/host/Replay.hpp"
#include <catch2/catch_all.hpp>
#include <catch2/catch_test_macros.hpp>
#include <cstdlib>
#include <ctime>
#include <exception>
#include <filesystem>

#include "depthai/depthai.hpp"
#include "depthai/pipeline/node/host/Record.hpp"
#include "depthai/utility/Compression.hpp"

constexpr unsigned int NUM_MSGS = 100;

class TestHelper {
   public:
    TestHelper() {
        srand(time(nullptr));
        testFolder = std::filesystem::path("/tmp/depthai_test_" + std::to_string(rand())).string();
        std::filesystem::create_directories(testFolder);
        std::filesystem::create_directories(testFolder + "/extracted");

        auto recordingFilenames = dai::utility::filenamesInTar(RECORDING_PATH);
        std::vector<std::string> recordingExtFiles;
        recordingExtFiles.reserve(recordingFilenames.size());
        for(const auto& filename : recordingFilenames) {
            recordingExtFiles.push_back(testFolder + "/extracted/" + filename);
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

    std::string testFolder;
};

TEST_CASE("ReplayMetadataOnly node") {
    {
        TestHelper helper;

        dai::Pipeline p(false);

        auto replayNode = p.create<dai::node::ReplayMetadataOnly>();
        replayNode->setReplayFile(helper.testFolder + "/extracted/IMU.mcap");
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
        replayNode->setReplayMetadataFile(helper.testFolder + "/extracted/CameraCAM_A.mcap");
        replayNode->setReplayVideoFile(helper.testFolder + "/extracted/CameraCAM_A.mp4");
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
