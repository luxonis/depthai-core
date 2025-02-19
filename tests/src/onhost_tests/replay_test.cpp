#include "depthai/pipeline/node/host/Replay.hpp"
#include <catch2/catch_all.hpp>
#include <catch2/catch_test_macros.hpp>
#include <cstdlib>
#include <ctime>
#include <filesystem>

#include "depthai/depthai.hpp"
#include "depthai/pipeline/node/host/Record.hpp"
#include "depthai/utility/Compression.hpp"

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
        std::filesystem::remove_all(testFolder);
    }

    std::string testFolder;
};

TEST_CASE("ReplayMetadataOnly node") {
    TestHelper helper;

    dai::Pipeline p(false);

    auto replayNode = p.create<dai::node::ReplayMetadataOnly>();
    replayNode->setReplayFile(helper.testFolder + "/extracted/IMU.mcap");
    replayNode->setLoop(false);

    p.start();

    p.wait();
}

TEST_CASE("ReplayVideo node") {
    TestHelper helper;

    dai::Pipeline p(false);

    auto replayNode = p.create<dai::node::ReplayVideo>();
    replayNode->setReplayMetadataFile(helper.testFolder + "/extracted/CameraCAM_A.mcap");
    replayNode->setReplayVideoFile(helper.testFolder + "/extracted/CameraCAM_A.mp4");
    replayNode->setLoop(false);

    p.start();

    p.wait();
}
