#include <catch2/catch_all.hpp>
#include <cstdlib>
#include <ctime>
#include <filesystem>

#include "depthai/common/CameraBoardSocket.hpp"
#include "depthai/depthai.hpp"
#include "depthai/pipeline/node/Camera.hpp"
#include "depthai/pipeline/node/host/Record.hpp"
#include "depthai/utility/Compression.hpp"

constexpr unsigned int NUM_MSGS = 50;

class TestHelper {
   public:
    TestHelper() {
        srand(time(nullptr));
        testFolder = std::filesystem::path("/tmp/depthai_test_" + std::to_string(rand())).string();
        std::filesystem::create_directories(testFolder);
        std::filesystem::create_directories(testFolder + "/recording_metadata");
        std::filesystem::create_directories(testFolder + "/recording_video");
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

TEST_CASE("MockIn IMU") {
    TestHelper helper;

    dai::Pipeline p;

    auto replayNode = p.create<dai::node::ReplayMetadataOnly>();
    replayNode->setReplayFile(helper.testFolder + "/extracted/IMU.mcap");
    replayNode->setLoop(false);

    auto imu = p.create<dai::node::IMU>();
    imu->enableIMUSensor(dai::IMUSensor::ACCELEROMETER_RAW, 500);
    imu->enableIMUSensor(dai::IMUSensor::GYROSCOPE_RAW, 400);
    imu->setBatchReportThreshold(100);

    replayNode->out.link(imu->mockIn);

    auto imuOut = imu->out.createOutputQueue();
    auto replayOut = replayNode->out.createOutputQueue();

    p.start();
    auto imuData = imuOut->get<dai::IMUData>();
    auto replayData = replayOut->get<dai::IMUData>();

    unsigned int i = 0;
    while(p.isRunning()) {
        if(i >= NUM_MSGS) break;
        if(imuData->getSequenceNum() > replayData->getSequenceNum()) {
            replayData = replayOut->get<dai::IMUData>();
        } else if(imuData->getSequenceNum() < replayData->getSequenceNum()) {
            imuData = imuOut->get<dai::IMUData>();
        } else {
            REQUIRE(imuData->str() == replayData->str());
            i++;
        }
    }
    p.stop();
}

