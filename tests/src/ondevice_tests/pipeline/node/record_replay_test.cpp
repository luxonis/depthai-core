#include <catch2/catch_all.hpp>
#include <chrono>
#include <cstdlib>
#include <ctime>
#include <filesystem>
#include <thread>

#include "../../src/utility/Platform.hpp"
#include "depthai/common/CameraBoardSocket.hpp"
#include "depthai/depthai.hpp"
#include "depthai/pipeline/node/Camera.hpp"
#include "depthai/pipeline/node/host/Record.hpp"
#include "depthai/utility/Compression.hpp"

using namespace std::literals::chrono_literals;

constexpr unsigned int NUM_MSGS = 200;

bool folderHasWritePermissions(const std::filesystem::path& folder) {
    // Check that the folder exists and is a directory.
    if(!std::filesystem::exists(folder) || !std::filesystem::is_directory(folder)) {
        return false;
    }

    // Create a temporary file path inside the folder.
    auto tempFile = folder / "temp_write_test_file.tmp";

    // Try to open the file for writing.
    std::ofstream ofs(tempFile, std::ios::out | std::ios::trunc);
    if(!ofs) {
        return false;
    }
    ofs.close();

    // Remove the temporary file.
    std::error_code ec;  // capture any error without throwing
    std::filesystem::remove(tempFile, ec);

    return true;
}

class TestHelper {
   public:
    TestHelper() {
        testFolder = dai::platform::getTempPath();
        std::filesystem::create_directories(testFolder);
        std::filesystem::create_directories(std::filesystem::path(testFolder).append("recording_metadata"));
        std::filesystem::create_directories(std::filesystem::path(testFolder).append("recording_video"));
        std::filesystem::create_directories(std::filesystem::path(testFolder).append("extracted"));

        if(!folderHasWritePermissions(testFolder)) {
            throw std::runtime_error("Test folder does not have write permissions: " + testFolder.string());
        }

        auto recordingFilenames = dai::utility::filenamesInTar(RECORDING_PATH);
        std::vector<std::filesystem::path> recordingExtFiles;
        recordingExtFiles.reserve(recordingFilenames.size());
        for(const auto& filename : recordingFilenames) {
            recordingExtFiles.push_back(std::filesystem::path(testFolder).append("extracted").append(filename));
        }
        dai::utility::untarFiles(RECORDING_PATH, recordingFilenames, recordingExtFiles);
    }

    ~TestHelper() {
        std::filesystem::remove_all(testFolder);
    }

    std::filesystem::path testFolder;
};

TEST_CASE("RecordMetadataOnly node") {
    TestHelper helper;

    dai::Pipeline p;

    auto imu = p.create<dai::node::IMU>();
    imu->enableIMUSensor(dai::IMUSensor::ACCELEROMETER_RAW, 500);
    imu->enableIMUSensor(dai::IMUSensor::GYROSCOPE_RAW, 400);
    imu->setBatchReportThreshold(100);

    auto recordNode = p.create<dai::node::RecordMetadataOnly>();
    recordNode->setRecordFile(std::filesystem::path(helper.testFolder).append("recording_metadata").append("metadata.mcap"));

    imu->out.link(recordNode->input);

    auto imuQ = imu->out.createOutputQueue();

    p.start();

    std::this_thread::sleep_for(5s);

    p.stop();

    REQUIRE(std::filesystem::exists(std::filesystem::path(helper.testFolder).append("recording_metadata").append("metadata.mcap")));
}

TEST_CASE("RecordVideo raw color") {
    TestHelper helper;

    dai::Pipeline p;

    auto cam = p.create<dai::node::Camera>()->build();
    auto camOut = cam->requestOutput({1280, 960}, dai::ImgFrame::Type::BGR888i);

    auto recordNode = p.create<dai::node::RecordVideo>();
    recordNode->setRecordMetadataFile(std::filesystem::path(helper.testFolder).append("recording_video").append("metadata_color.mcap"));
    recordNode->setRecordVideoFile(std::filesystem::path(helper.testFolder).append("recording_video").append("video_color.mp4"));

    camOut->link(recordNode->input);

    auto camQ = camOut->createOutputQueue();

    p.start();

    std::this_thread::sleep_for(5s);

    p.stop();

    REQUIRE(std::filesystem::exists(std::filesystem::path(helper.testFolder).append("recording_video").append("metadata_color.mcap")));
    REQUIRE(std::filesystem::exists(std::filesystem::path(helper.testFolder).append("recording_video").append("video_color.mp4")));
}

TEST_CASE("RecordVideo raw gray") {
    TestHelper helper;

    dai::Pipeline p;

    auto cam = p.create<dai::node::Camera>()->build();
    auto camOut = cam->requestOutput({1280, 960}, dai::ImgFrame::Type::GRAY8);

    auto recordNode = p.create<dai::node::RecordVideo>();
    recordNode->setRecordMetadataFile(std::filesystem::path(helper.testFolder).append("recording_video").append("metadata_gray.mcap"));
    recordNode->setRecordVideoFile(std::filesystem::path(helper.testFolder).append("recording_video").append("video_gray.mp4"));

    camOut->link(recordNode->input);

    auto camQ = camOut->createOutputQueue();

    p.start();

    std::this_thread::sleep_for(5s);

    p.stop();

    REQUIRE(std::filesystem::exists(std::filesystem::path(helper.testFolder).append("recording_video").append("metadata_gray.mcap")));
    REQUIRE(std::filesystem::exists(std::filesystem::path(helper.testFolder).append("recording_video").append("video_gray.mp4")));
}

TEST_CASE("RecordVideo encoded h264") {
    TestHelper helper;

    dai::Pipeline p;

    auto cam = p.create<dai::node::Camera>()->build();
    auto camOut = cam->requestOutput({1280, 960}, dai::ImgFrame::Type::NV12);
    auto videoEncoder = p.create<dai::node::VideoEncoder>();

    auto recordNode = p.create<dai::node::RecordVideo>();
    recordNode->setRecordMetadataFile(std::filesystem::path(helper.testFolder).append("recording_video").append("metadata_h264.mcap"));
    recordNode->setRecordVideoFile(std::filesystem::path(helper.testFolder).append("recording_video").append("video_h264.mp4"));

    videoEncoder->setProfile(dai::VideoEncoderProperties::Profile::H264_MAIN);

    camOut->link(videoEncoder->input);
    videoEncoder->out.link(recordNode->input);

    auto camQ = camOut->createOutputQueue();

    p.start();

    std::this_thread::sleep_for(5s);

    p.stop();

    REQUIRE(std::filesystem::exists(std::filesystem::path(helper.testFolder).append("recording_video").append("metadata_h264.mcap")));
    REQUIRE(std::filesystem::exists(std::filesystem::path(helper.testFolder).append("recording_video").append("video_h264.mp4")));
}

TEST_CASE("RecordVideo encoded mjpeg") {
    TestHelper helper;

    dai::Pipeline p;

    auto cam = p.create<dai::node::Camera>()->build();
    auto camOut = cam->requestOutput({1280, 960}, dai::ImgFrame::Type::NV12);
    auto videoEncoder = p.create<dai::node::VideoEncoder>();

    auto recordNode = p.create<dai::node::RecordVideo>();
    recordNode->setRecordMetadataFile(std::filesystem::path(helper.testFolder).append("recording_video").append("metadata_mjpeg.mcap"));
    recordNode->setRecordVideoFile(std::filesystem::path(helper.testFolder).append("recording_video").append("video_mjpeg.mp4"));

    videoEncoder->setProfile(dai::VideoEncoderProperties::Profile::MJPEG);

    camOut->link(videoEncoder->input);
    videoEncoder->out.link(recordNode->input);

    auto camQ = camOut->createOutputQueue();

    p.start();

    std::this_thread::sleep_for(5s);

    p.stop();

    REQUIRE(std::filesystem::exists(std::filesystem::path(helper.testFolder).append("recording_video").append("metadata_mjpeg.mcap")));
    REQUIRE(std::filesystem::exists(std::filesystem::path(helper.testFolder).append("recording_video").append("video_mjpeg.mp4")));
}

TEST_CASE("MockIn Camera") {
    TestHelper helper;

    dai::Pipeline p;

    auto replayNode = p.create<dai::node::ReplayVideo>();
    replayNode->setReplayMetadataFile(std::filesystem::path(helper.testFolder).append("extracted").append("CameraCAM_A.mcap"));
    replayNode->setReplayVideoFile(std::filesystem::path(helper.testFolder).append("extracted").append("CameraCAM_A.mp4"));
    replayNode->setLoop(false);

    auto cam = p.create<dai::node::Camera>()->build(dai::CameraBoardSocket::CAM_A, *replayNode);
    auto camOut = cam->requestOutput({2016, 1520});

    auto camQ = camOut->createOutputQueue();
    auto replayQ = replayNode->out.createOutputQueue();

    p.start();
    auto camData = camQ->get<dai::ImgFrame>();
    auto replayData = replayQ->get<dai::ImgFrame>();

    unsigned int i = 0;
    while(p.isRunning()) {
        if(i >= NUM_MSGS) break;
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
            i++;
        }
    }
    REQUIRE(i == NUM_MSGS);
}

TEST_CASE("MockIn IMU") {
    TestHelper helper;

    dai::Pipeline p;

    auto replayNode = p.create<dai::node::ReplayMetadataOnly>();
    replayNode->setReplayFile(std::filesystem::path(helper.testFolder).append("extracted").append("IMU.mcap"));
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
