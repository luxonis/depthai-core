#include <catch2/catch_all.hpp>
#include <catch2/catch_test_macros.hpp>
#include <iostream>

#include "depthai/device/Device.hpp"
#include "depthai/pipeline/Pipeline.hpp"
#include "depthai/pipeline/datatype/EncodedFrame.hpp"
#include "depthai/pipeline/datatype/ImgFrame.hpp"
#include "depthai/pipeline/node/ColorCamera.hpp"
#include "depthai/pipeline/node/VideoEncoder.hpp"
#include "depthai/pipeline/node/XLinkOut.hpp"
#include "depthai/properties/VideoEncoderProperties.hpp"

dai::Pipeline getPipeline(dai::VideoEncoderProperties::Profile profile, unsigned int quality, bool lossless, unsigned int bitrate) {
    dai::Pipeline pipeline;
    auto camNode = pipeline.create<dai::node::ColorCamera>();
    auto encNode = pipeline.create<dai::node::VideoEncoder>();
    auto xlinkOut = pipeline.create<dai::node::XLinkOut>();
    camNode->video.link(encNode->input);
    encNode->out.link(xlinkOut->input);

    camNode->setVideoSize(1280, 720);
    encNode->setProfile(profile);
    encNode->setBitrate(bitrate);
    encNode->setQuality(quality);
    encNode->setLossless(lossless);
    encNode->setKeyframeFrequency(30);
    xlinkOut->setStreamName("out");

    return pipeline;
}

TEST_CASE("OLD_OUTPUT") {
    dai::Pipeline pipeline;
    auto camNode = pipeline.create<dai::node::ColorCamera>();
    auto encNode = pipeline.create<dai::node::VideoEncoder>();
    camNode->video.link(encNode->input);

    camNode->setVideoSize(1280, 720);
    encNode->setProfile(dai::VideoEncoderProperties::Profile::H264_MAIN);

    auto outputQueue = encNode->bitstream.createOutputQueue();
    pipeline.start();
    for(int i = 0; i < 100; ++i) {
        REQUIRE_NOTHROW(outputQueue->get<dai::ImgFrame>());
    }
}

TEST_CASE("JPEG_ENCODING_LOSSLESS") {
    dai::Pipeline pipeline;
    auto camNode = pipeline.create<dai::node::ColorCamera>();
    auto encNode = pipeline.create<dai::node::VideoEncoder>();
    camNode->video.link(encNode->input);

    dai::VideoEncoderProperties::Profile profile = dai::VideoEncoderProperties::Profile::MJPEG;
    unsigned int quality = 30;
    bool lossless = true;
    unsigned int bitrate = 0;

    camNode->setVideoSize(1280, 720);
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
        REQUIRE(encfrm->getLossless() == true);
        REQUIRE(encfrm->getQuality() == 30);
    }
}

TEST_CASE("JPEG_ENCODING_LOSSY") {
    dai::Pipeline pipeline;
    auto camNode = pipeline.create<dai::node::ColorCamera>();
    auto encNode = pipeline.create<dai::node::VideoEncoder>();
    camNode->video.link(encNode->input);

    dai::VideoEncoderProperties::Profile profile = dai::VideoEncoderProperties::Profile::MJPEG;
    unsigned int quality = 30;
    bool lossless = false;
    unsigned int bitrate = 0;

    camNode->setVideoSize(1280, 720);
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
    auto camNode = pipeline.create<dai::node::ColorCamera>();
    auto encNode = pipeline.create<dai::node::VideoEncoder>();
    camNode->video.link(encNode->input);

    dai::VideoEncoderProperties::Profile profile = dai::VideoEncoderProperties::Profile::H264_HIGH;
    unsigned int quality = 30;
    bool lossless = false;
    unsigned int bitrate = 8500000;

    camNode->setVideoSize(1280, 720);
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
    auto camNode = pipeline.create<dai::node::ColorCamera>();
    auto encNode = pipeline.create<dai::node::VideoEncoder>();
    camNode->video.link(encNode->input);

    dai::VideoEncoderProperties::Profile profile = dai::VideoEncoderProperties::Profile::H265_MAIN;
    unsigned int quality = 30;
    bool lossless = false;
    unsigned int bitrate = 8500000;

    camNode->setVideoSize(1280, 720);
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