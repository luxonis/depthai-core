#include <catch2/catch_all.hpp>
#include <cstdlib>
#include <ctime>
#include <filesystem>

#include "depthai/common/CameraBoardSocket.hpp"
#include "depthai/depthai.hpp"
#include "depthai/pipeline/node/Camera.hpp"
#include "depthai/pipeline/node/host/Record.hpp"
#include "depthai/utility/Compression.hpp"

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

TEST_CASE("RecordMetadataOnly node") {
    TestHelper helper;

    dai::Pipeline p;

    auto imu = p.create<dai::node::IMU>();
    imu->enableIMUSensor(dai::IMUSensor::ACCELEROMETER_RAW, 500);
    imu->enableIMUSensor(dai::IMUSensor::GYROSCOPE_RAW, 400);
    imu->setBatchReportThreshold(100);

    auto recordNode = p.create<dai::node::RecordMetadataOnly>();
    recordNode->setRecordFile(helper.testFolder + "/recording_metadata/metadata.mcap");

    imu->out.link(recordNode->input);

    p.start();

    std::this_thread::sleep_for(std::chrono::seconds(5));

    p.stop();

    REQUIRE(std::filesystem::exists(helper.testFolder + "/recording_metadata/metadata.mcap"));
}

TEST_CASE("RecordVideo node") {
    TestHelper helper;

    dai::Pipeline p;

    auto cam = p.create<dai::node::Camera>()->build();
    auto camOut = cam->requestOutput({1280, 720});

    auto recordNode = p.create<dai::node::RecordVideo>();
    recordNode->setRecordMetadataFile(helper.testFolder + "/recording_video/metadata.mcap");
    recordNode->setRecordVideoFile(helper.testFolder + "/recording_video/video.mp4");

    camOut->link(recordNode->input);

    p.start();

    std::this_thread::sleep_for(std::chrono::seconds(5));

    p.stop();

    REQUIRE(std::filesystem::exists(helper.testFolder + "/recording_video/metadata.mcap"));
    REQUIRE(std::filesystem::exists(helper.testFolder + "/recording_video/video.mp4"));
}

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

    while(p.isRunning()) {
        if(imuData->getSequenceNum() > replayData->getSequenceNum()) {
            replayData = replayOut->get<dai::IMUData>();
        } else if(imuData->getSequenceNum() < replayData->getSequenceNum()) {
            imuData = imuOut->get<dai::IMUData>();
        } else {
            REQUIRE(imuData->str() == replayData->str());
        }
    }
}

TEST_CASE("MockIn Camera") {
    TestHelper helper;

    dai::Pipeline p;

    auto replayNode = p.create<dai::node::ReplayVideo>();
    replayNode->setReplayMetadataFile(helper.testFolder + "/extracted/CameraCAM_A.mcap");
    replayNode->setReplayVideoFile(helper.testFolder + "/extracted/CameraCAM_A.mp4");
    replayNode->setLoop(false);

    auto cam = p.create<dai::node::Camera>()->build(dai::CameraBoardSocket::CAM_A, *replayNode);
    auto camOut = cam->requestOutput({2016, 1520});

    auto camQ = camOut->createOutputQueue();
    auto replayQ = replayNode->out.createOutputQueue();

    p.start();
    auto camData = camQ->get<dai::ImgFrame>();
    auto replayData = replayQ->get<dai::ImgFrame>();

    while(p.isRunning()) {
        if(camData->getSequenceNum() > replayData->getSequenceNum()) {
            replayData = replayQ->get<dai::ImgFrame>();
        } else if(camData->getSequenceNum() < replayData->getSequenceNum()) {
            camData = camQ->get<dai::ImgFrame>();
        } else {
            REQUIRE(camData->getTimestamp() == replayData->getTimestamp());
            REQUIRE(camData->getTimestampDevice() == replayData->getTimestampDevice());
            REQUIRE(camData->getWidth() == replayData->getWidth());
            REQUIRE(camData->getHeight() == replayData->getHeight());
            REQUIRE(camData->getData().size() == replayData->getData().size());
        }
    }
}
