#include <catch2/catch_all.hpp>
#include <chrono>
#include <opencv2/opencv.hpp>
#include <thread>

#include "depthai/depthai.hpp"
#include "depthai/pipeline/node/host/Record.hpp"
#include "depthai/pipeline/node/host/Replay.hpp"

static constexpr unsigned int NUM_FRAMES = 350;

double calculateEncodedVideoPSNR(const std::filesystem::path& originalVideo, const std::filesystem::path& encodedVideo) {
    cv::VideoCapture originalCapture(originalVideo.string());
    cv::VideoCapture encodedCapture(encodedVideo.string());
    REQUIRE((originalCapture.isOpened() && encodedCapture.isOpened()));
    int frameCount = 0;
    double psnrSum = 0.0;
    cv::Mat originalFrame, encodedFrame;
    while(originalCapture.read(originalFrame) && encodedCapture.read(encodedFrame)) {
        psnrSum += cv::PSNR(originalFrame, encodedFrame);
        ++frameCount;
    }
    return psnrSum / frameCount;
}

void recordEncodedVideo(const std::filesystem::path& path, const std::filesystem::path& encodedPath, const dai::VideoEncoderProperties& properties) {
    dai::Pipeline pipeline;

    auto replayVideoNode = pipeline.create<dai::node::ReplayVideo>();
    replayVideoNode->setReplayVideoFile(path);
    replayVideoNode->setOutFrameType(dai::ImgFrame::Type::NV12);
    replayVideoNode->setLoop(false);
    replayVideoNode->setFps(properties.frameRate);

    auto videoEncoderNode = pipeline.create<dai::node::VideoEncoder>();
    videoEncoderNode->setProfile(properties.profile);
    videoEncoderNode->setBitrate(properties.bitrate);
    videoEncoderNode->setQuality(properties.quality);
    videoEncoderNode->setLossless(properties.lossless);
    videoEncoderNode->setFrameRate(properties.frameRate);
    videoEncoderNode->setRateControlMode(properties.rateCtrlMode);

    auto recordVideoNode = pipeline.create<dai::node::RecordVideo>();
    recordVideoNode->setRecordVideoFile(encodedPath);
    recordVideoNode->setFps(properties.frameRate);

    replayVideoNode->out.link(videoEncoderNode->input);
    videoEncoderNode->out.link(recordVideoNode->input);

    auto replayQueue = replayVideoNode->out.createOutputQueue();

    pipeline.start();
    while(pipeline.isRunning()) {
        try {
            auto replayData = replayQueue->get<dai::ImgFrame>();
            if(replayData->sequenceNum == NUM_FRAMES) {
                break;
            }
        } catch(...) {
            break;
        }
    }
}

TEST_CASE("Test VideoEncoder node H264_HIGH") {
    std::filesystem::path path(VIDEO_PATH);
    REQUIRE(std::filesystem::exists(path));

    dai::VideoEncoderProperties properties;
    properties.profile = dai::VideoEncoderProperties::Profile::H264_HIGH;
    std::filesystem::path encodedPath = path.parent_path() / (path.stem().string() + "encoded" + path.extension().string());

    // Test bitrate setting
    properties.frameRate = 25;
    properties.bitrate = 1000000;
    recordEncodedVideo(path, encodedPath, properties);
    REQUIRE(std::filesystem::exists(encodedPath));
    auto encodedFileSize1 = std::filesystem::file_size(encodedPath);
    double psnr1 = calculateEncodedVideoPSNR(VIDEO_PATH, encodedPath);

    properties.bitrate = 2500000;
    recordEncodedVideo(path, encodedPath, properties);
    REQUIRE(std::filesystem::exists(encodedPath));
    auto encodedFileSize2 = std::filesystem::file_size(encodedPath);
    double psnr2 = calculateEncodedVideoPSNR(VIDEO_PATH, encodedPath);

    REQUIRE(psnr1 < psnr2);
    REQUIRE(encodedFileSize1 < encodedFileSize2);

    // Test VBR mode
    properties.bitrate = 0;
    properties.rateCtrlMode = dai::VideoEncoderProperties::RateControlMode::VBR;
    recordEncodedVideo(path, encodedPath, properties);
    REQUIRE(std::filesystem::exists(encodedPath));
    auto encodedFileSize = std::filesystem::file_size(encodedPath);
    double psnr = calculateEncodedVideoPSNR(VIDEO_PATH, encodedPath);

    // PSNR Bellow 30 dB indicates significant degredation
    REQUIRE(psnr > 30.0);
    REQUIRE(encodedFileSize > 0);

    // Clear the encoded video file
    std::filesystem::remove(encodedPath);
}

TEST_CASE("Test VideoEncoder node H264_MAIN") {
    std::filesystem::path path(VIDEO_PATH);
    REQUIRE(std::filesystem::exists(path));

    dai::VideoEncoderProperties properties;
    properties.profile = dai::VideoEncoderProperties::Profile::H264_MAIN;
    std::filesystem::path encodedPath = path.parent_path() / (path.stem().string() + "encoded" + path.extension().string());

    // Test bitrate setting
    properties.frameRate = 25;
    properties.bitrate = 1000000;
    recordEncodedVideo(path, encodedPath, properties);
    REQUIRE(std::filesystem::exists(encodedPath));
    auto encodedFileSize1 = std::filesystem::file_size(encodedPath);
    double psnr1 = calculateEncodedVideoPSNR(VIDEO_PATH, encodedPath);

    properties.bitrate = 2500000;
    recordEncodedVideo(path, encodedPath, properties);
    REQUIRE(std::filesystem::exists(encodedPath));
    auto encodedFileSize2 = std::filesystem::file_size(encodedPath);
    double psnr2 = calculateEncodedVideoPSNR(VIDEO_PATH, encodedPath);

    REQUIRE(psnr1 < psnr2);
    REQUIRE(encodedFileSize1 < encodedFileSize2);

    // Test VBR mode
    properties.bitrate = 0;
    properties.rateCtrlMode = dai::VideoEncoderProperties::RateControlMode::VBR;
    recordEncodedVideo(path, encodedPath, properties);
    REQUIRE(std::filesystem::exists(encodedPath));
    auto encodedFileSize = std::filesystem::file_size(encodedPath);
    double psnr = calculateEncodedVideoPSNR(VIDEO_PATH, encodedPath);

    // PSNR Bellow 30 dB indicates significant degredation
    REQUIRE(psnr > 30.0);
    REQUIRE(encodedFileSize > 0);

    // Clear the encoded video file
    std::filesystem::remove(encodedPath);
}

TEST_CASE("Test VideoEncoder node H264_BASELINE") {
    std::filesystem::path path(VIDEO_PATH);
    REQUIRE(std::filesystem::exists(path));

    dai::VideoEncoderProperties properties;
    properties.profile = dai::VideoEncoderProperties::Profile::H264_BASELINE;
    std::filesystem::path encodedPath = path.parent_path() / (path.stem().string() + "encoded" + path.extension().string());

    // Test bitrate setting
    properties.frameRate = 25;
    properties.bitrate = 1000000;
    recordEncodedVideo(path, encodedPath, properties);
    REQUIRE(std::filesystem::exists(encodedPath));
    auto encodedFileSize1 = std::filesystem::file_size(encodedPath);
    double psnr1 = calculateEncodedVideoPSNR(VIDEO_PATH, encodedPath);

    properties.bitrate = 2500000;
    recordEncodedVideo(path, encodedPath, properties);
    REQUIRE(std::filesystem::exists(encodedPath));
    auto encodedFileSize2 = std::filesystem::file_size(encodedPath);
    double psnr2 = calculateEncodedVideoPSNR(VIDEO_PATH, encodedPath);

    REQUIRE(psnr1 < psnr2);
    REQUIRE(encodedFileSize1 < encodedFileSize2);

    // Test VBR mode
    properties.bitrate = 0;
    properties.rateCtrlMode = dai::VideoEncoderProperties::RateControlMode::VBR;
    recordEncodedVideo(path, encodedPath, properties);
    REQUIRE(std::filesystem::exists(encodedPath));
    auto encodedFileSize = std::filesystem::file_size(encodedPath);
    double psnr = calculateEncodedVideoPSNR(VIDEO_PATH, encodedPath);

    // PSNR Bellow 30 dB indicates significant degredation
    REQUIRE(psnr > 30.0);
    REQUIRE(encodedFileSize > 0);

    // Clear the encoded video file
    std::filesystem::remove(encodedPath);
}
/*
TEST_CASE("Test VideoEncoder node H265_MAIN") {
    std::filesystem::path path(VIDEO_PATH);
    REQUIRE(std::filesystem::exists(path));

    dai::VideoEncoderProperties properties;
    properties.profile = dai::VideoEncoderProperties::Profile::H265_MAIN;
    std::filesystem::path encodedPath = path.parent_path() / (path.stem().string() + "encoded" + path.extension().string());

    // Test bitrate setting
    properties.frameRate = 25;
    properties.bitrate = 1000000;
    recordEncodedVideo(path, encodedPath, properties);
    REQUIRE(std::filesystem::exists(encodedPath));
    auto encodedFileSize1 = std::filesystem::file_size(encodedPath);
    double psnr1 = calculateEncodedVideoPSNR(VIDEO_PATH, encodedPath);

    properties.bitrate = 2500000;
    recordEncodedVideo(path, encodedPath, properties);
    REQUIRE(std::filesystem::exists(encodedPath));
    auto encodedFileSize2 = std::filesystem::file_size(encodedPath);
    double psnr2 = calculateEncodedVideoPSNR(VIDEO_PATH, encodedPath);

    REQUIRE(psnr1 < psnr2);
    REQUIRE(encodedFileSize1 < encodedFileSize2);

    // Test VBR mode
    properties.bitrate = 0;
    properties.rateCtrlMode = dai::VideoEncoderProperties::RateControlMode::VBR;
    recordEncodedVideo(path, encodedPath, properties);
    REQUIRE(std::filesystem::exists(encodedPath));
    auto encodedFileSize = std::filesystem::file_size(encodedPath);
    double psnr = calculateEncodedVideoPSNR(VIDEO_PATH, encodedPath);

    // PSNR Bellow 30 dB indicates significant degredation
    REQUIRE(psnr > 30.0);
    REQUIRE(encodedFileSize > 0);

    // Clear the encoded video file
    std::filesystem::remove(encodedPath);
}

TEST_CASE("Test VideoEncoder node MJPEG") {
    std::filesystem::path path(VIDEO_PATH);
    REQUIRE(std::filesystem::exists(path));

    dai::VideoEncoderProperties properties;
    properties.profile = dai::VideoEncoderProperties::Profile::MJPEG;
    std::filesystem::path encodedPath = path.parent_path() / (path.stem().string() + "encoded" + path.extension().string());

    // Test bitrate setting
    properties.frameRate = 25;
    properties.bitrate = 0;
    recordEncodedVideo(path, encodedPath, properties);
    REQUIRE(std::filesystem::exists(encodedPath));
    auto encodedFileSize1 = std::filesystem::file_size(encodedPath);
    double psnr1 = calculateEncodedVideoPSNR(VIDEO_PATH, encodedPath);

    properties.bitrate = 0;
    recordEncodedVideo(path, encodedPath, properties);
    REQUIRE(std::filesystem::exists(encodedPath));
    auto encodedFileSize2 = std::filesystem::file_size(encodedPath);
    double psnr2 = calculateEncodedVideoPSNR(VIDEO_PATH, encodedPath);

    std::cout << psnr1 << "dB   " << psnr2 << "dB\n"; // TO DO Remove
    std::cout << encodedFileSize1 << "bytes    " << encodedFileSize2 << "bytes\n"; // TO DO Remove
    REQUIRE(psnr1 < psnr2);
    REQUIRE(encodedFileSize1 < encodedFileSize2);

    // Test lossless mode
    properties.lossless = true;
    recordEncodedVideo(path, encodedPath, properties);
    REQUIRE(std::filesystem::exists(encodedPath));
    auto encodedFileSize3 = std::filesystem::file_size(encodedPath);
    double psnr3 = calculateEncodedVideoPSNR(VIDEO_PATH, encodedPath);

    std::cout << psnr3 << "dB\n"; // TO DO Remove
    std::cout << encodedFileSize3 << "bytes\n"; // TO DO Remove
    REQUIRE(psnr2 < psnr3);
    REQUIRE(encodedFileSize2 < encodedFileSize3);

    // Test VBR mode
    properties.bitrate = 0;
    properties.rateCtrlMode = dai::VideoEncoderProperties::RateControlMode::VBR;
    recordEncodedVideo(path, encodedPath, properties);
    REQUIRE(std::filesystem::exists(encodedPath));
    auto encodedFileSize = std::filesystem::file_size(encodedPath);
    double psnr = calculateEncodedVideoPSNR(VIDEO_PATH, encodedPath);

    // PSNR Bellow 30 dB indicates significant degredation
    std::cout << psnr << "dB\n"; // TO DO Remove
    std::cout << encodedFileSize << "bytes\n"; // TO DO Remove
    REQUIRE(psnr > 30.0);
    REQUIRE(encodedFileSize > 0);

    // Clear the encoded video file
    std::filesystem::remove(encodedPath);
}
*/
TEST_CASE("Test VideoEncoder H264 profile comparison") {
    std::filesystem::path path(VIDEO_PATH);
    REQUIRE(std::filesystem::exists(path));

    dai::VideoEncoderProperties properties;
    std::filesystem::path encodedPath = path.parent_path() / (path.stem().string() + "encoded" + path.extension().string());

    // Compare psnr and file sizes between different H264 profiles
    properties.frameRate = 25;
    properties.bitrate = 2500000;
    properties.profile = dai::VideoEncoderProperties::Profile::H264_HIGH;
    recordEncodedVideo(path, encodedPath, properties);
    REQUIRE(std::filesystem::exists(encodedPath));
    auto encodedFileSize1 = std::filesystem::file_size(encodedPath);
    double psnr1 = calculateEncodedVideoPSNR(VIDEO_PATH, encodedPath);

    properties.profile = dai::VideoEncoderProperties::Profile::H264_MAIN;
    recordEncodedVideo(path, encodedPath, properties);
    REQUIRE(std::filesystem::exists(encodedPath));
    auto encodedFileSize2 = std::filesystem::file_size(encodedPath);
    double psnr2 = calculateEncodedVideoPSNR(VIDEO_PATH, encodedPath);

    properties.profile = dai::VideoEncoderProperties::Profile::H264_BASELINE;
    recordEncodedVideo(path, encodedPath, properties);
    REQUIRE(std::filesystem::exists(encodedPath));
    auto encodedFileSize3 = std::filesystem::file_size(encodedPath);
    double psnr3 = calculateEncodedVideoPSNR(VIDEO_PATH, encodedPath);

    // PSNR should be in the following order:  H264_HIGH > H264_MAIN > H264_BASELINE
    // File size should be in the following order:  H264_BASELINE > H264_MAIN > H264_HIGH
    REQUIRE(psnr1 > psnr2);
    REQUIRE(psnr2 > psnr3);
    REQUIRE(encodedFileSize1 < encodedFileSize2);
    REQUIRE(encodedFileSize2 < encodedFileSize3);

    // Clear the encoded video file
    std::filesystem::remove(encodedPath);
}
