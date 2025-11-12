#include <catch2/catch_all.hpp>
#include <catch2/catch_test_macros.hpp>
#include <fstream>
#include <iostream>
#include <opencv2/opencv.hpp>

#include "depthai/pipeline/Pipeline.hpp"
#include "depthai/pipeline/datatype/EncodedFrame.hpp"
#include "depthai/pipeline/datatype/ImgFrame.hpp"
#include "depthai/pipeline/node/Camera.hpp"
#include "depthai/pipeline/node/VideoEncoder.hpp"
#include "depthai/properties/VideoEncoderProperties.hpp"

TEST_CASE("OLD_OUTPUT") {
    dai::Pipeline pipeline;
    auto camNode = pipeline.create<dai::node::Camera>()->build();
    auto camOut = camNode->requestOutput({640, 480}, dai::ImgFrame::Type::NV12);
    auto encNode = pipeline.create<dai::node::VideoEncoder>();
    camOut->link(encNode->input);

    encNode->setProfile(dai::VideoEncoderProperties::Profile::H264_MAIN);

    auto outputQueue = encNode->bitstream.createOutputQueue();
    pipeline.start();
    for(int i = 0; i < 100; ++i) {
        REQUIRE_NOTHROW(outputQueue->get<dai::ImgFrame>());
    }
}

TEST_CASE("JPEG_ENCODING_LOSSLESS") {
    dai::Pipeline pipeline;
    if(pipeline.getDefaultDevice()->getPlatform() == dai::Platform::RVC4) {
        return;
    }

    auto camNode = pipeline.create<dai::node::Camera>()->build();
    auto camOut = camNode->requestOutput({640, 480}, dai::ImgFrame::Type::NV12);
    auto encNode = pipeline.create<dai::node::VideoEncoder>();
    camOut->link(encNode->input);

    dai::VideoEncoderProperties::Profile profile = dai::VideoEncoderProperties::Profile::MJPEG;
    unsigned int quality = 30;
    bool lossless = true;
    unsigned int bitrate = 0;

    encNode->setProfile(profile);
    encNode->setBitrate(bitrate);
    encNode->setQuality(quality);
    encNode->setLossless(lossless);
    encNode->setKeyframeFrequency(30);

    auto outputQueue = encNode->out.createOutputQueue();
    auto camOutputQueue = camOut->createOutputQueue();
    pipeline.start();
    for(int i = 0; i < 100; ++i) {
        auto encfrm = outputQueue->get<dai::EncodedFrame>();
        auto origFrame = camOutputQueue->get<dai::ImgFrame>();
        REQUIRE(encfrm->getProfile() == dai::EncodedFrame::Profile::JPEG);
        REQUIRE(encfrm->getLossless() == true);
        REQUIRE(encfrm->getQuality() == 30);

        // Encoded and original frames should be identical
        std::string data;
        std::stringstream ss;
        ss.write((const char*)encfrm->getData().data(), encfrm->getData().size());
        data = ss.str();
        std::filesystem::path path("encoded");
        std::ofstream fileStream(path, std::ios::binary);
        fileStream.write(data.data(), data.size());

        ss.write((const char*)origFrame->getData().data(), origFrame->getData().size());
        data = ss.str();
        path = std::filesystem::path("original");
        fileStream = std::ofstream(path, std::ios::binary);
        fileStream.write(data.data(), data.size());

        cv::VideoCapture originalCapture("original", cv::CAP_FFMPEG);
        cv::VideoCapture encodedCapture("encoded", cv::CAP_FFMPEG);
        cv::Mat originalFrame, encodedFrame;
        REQUIRE((originalCapture.read(originalFrame) && encodedCapture.read(encodedFrame)));
        REQUIRE(std::equal(originalFrame.begin<uchar>(), originalFrame.end<uchar>(), encodedFrame.begin<uchar>()));
    }
    std::filesystem::remove("original");
    std::filesystem::remove("encoded");
}

TEST_CASE("JPEG_ENCODING_LOSSY") {
    dai::Pipeline pipeline;
    auto camNode = pipeline.create<dai::node::Camera>()->build();
    auto camOut = camNode->requestOutput({640, 480}, dai::ImgFrame::Type::NV12);
    auto encNode = pipeline.create<dai::node::VideoEncoder>();
    camOut->link(encNode->input);

    dai::VideoEncoderProperties::Profile profile = dai::VideoEncoderProperties::Profile::MJPEG;
    unsigned int quality = 30;
    bool lossless = false;
    unsigned int bitrate = 0;

    encNode->setProfile(profile);
    encNode->setBitrate(bitrate);
    encNode->setQuality(quality);
    encNode->setLossless(lossless);
    encNode->setKeyframeFrequency(30);

    auto outputQueue = encNode->out.createOutputQueue();
    pipeline.start();
    for(int i = 0; i < 100; ++i) {
        auto encfrm = outputQueue->get<dai::EncodedFrame>();
        REQUIRE(encfrm->getProfile() == dai::EncodedFrame::Profile::JPEG);
        REQUIRE(encfrm->getLossless() == false);
        REQUIRE(encfrm->getQuality() == 30);
    }
}

TEST_CASE("AVC_ENCODING") {
    dai::Pipeline pipeline;
    auto camNode = pipeline.create<dai::node::Camera>()->build();
    auto camOut = camNode->requestOutput({640, 480}, dai::ImgFrame::Type::NV12);
    auto encNode = pipeline.create<dai::node::VideoEncoder>();
    camOut->link(encNode->input);

    dai::VideoEncoderProperties::Profile profile = dai::VideoEncoderProperties::Profile::H264_HIGH;
    unsigned int quality = 30;
    bool lossless = false;
    unsigned int bitrate = 8500000;

    encNode->setProfile(profile);
    encNode->setBitrate(bitrate);
    encNode->setQuality(quality);
    encNode->setLossless(lossless);
    encNode->setKeyframeFrequency(30);

    auto outputQueue = encNode->out.createOutputQueue();
    pipeline.start();
    for(int i = 0; i < 100; ++i) {
        auto encfrm = outputQueue->get<dai::EncodedFrame>();
        REQUIRE(encfrm->getProfile() == dai::EncodedFrame::Profile::AVC);
        REQUIRE(encfrm->getLossless() == false);
        if(i % 30 == 0) REQUIRE(encfrm->getFrameType() == dai::EncodedFrame::FrameType::I);
        REQUIRE(encfrm->getQuality() == 30);
        REQUIRE(encfrm->getBitrate() == 8500000);
    }
}

TEST_CASE("HEVC_ENCODING") {
    dai::Pipeline pipeline;
    auto camNode = pipeline.create<dai::node::Camera>()->build();
    auto camOut = camNode->requestOutput({640, 480}, dai::ImgFrame::Type::NV12);
    auto encNode = pipeline.create<dai::node::VideoEncoder>();
    camOut->link(encNode->input);

    dai::VideoEncoderProperties::Profile profile = dai::VideoEncoderProperties::Profile::H265_MAIN;
    unsigned int quality = 30;
    bool lossless = false;
    unsigned int bitrate = 8500000;

    encNode->setProfile(profile);
    encNode->setBitrate(bitrate);
    encNode->setQuality(quality);
    encNode->setLossless(lossless);
    encNode->setKeyframeFrequency(30);

    auto outputQueue = encNode->out.createOutputQueue();
    pipeline.start();
    for(int i = 0; i < 100; ++i) {
        auto encfrm = outputQueue->get<dai::EncodedFrame>();
        REQUIRE(encfrm->getProfile() == dai::EncodedFrame::Profile::HEVC);
        REQUIRE(encfrm->getLossless() == false);
        if(i % 30 == 0) REQUIRE(encfrm->getFrameType() == dai::EncodedFrame::FrameType::I);
        REQUIRE(encfrm->getQuality() == 30);
        REQUIRE(encfrm->getBitrate() == 8500000);
    }
}
