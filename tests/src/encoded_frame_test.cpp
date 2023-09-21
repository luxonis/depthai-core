#include <catch2/catch_all.hpp>
#include <catch2/catch_test_macros.hpp>
#include <iostream>

#include "depthai-shared/datatype/RawEncodedFrame.hpp"
#include "depthai-shared/properties/VideoEncoderProperties.hpp"
#include "depthai/device/Device.hpp"
#include "depthai/pipeline/Pipeline.hpp"
#include "depthai/pipeline/datatype/EncodedFrame.hpp"
#include "depthai/pipeline/datatype/ImgFrame.hpp"
#include "depthai/pipeline/node/ColorCamera.hpp"
#include "depthai/pipeline/node/VideoEncoder.hpp"
#include "depthai/pipeline/node/XLinkOut.hpp"

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
    auto xlinkOut = pipeline.create<dai::node::XLinkOut>();
    camNode->video.link(encNode->input);
    encNode->bitstream.link(xlinkOut->input);

    camNode->setVideoSize(1280, 720);
    encNode->setProfile(dai::VideoEncoderProperties::Profile::H264_MAIN);
    xlinkOut->setStreamName("out");

    dai::Device device(pipeline);

    auto outQ = device.getOutputQueue("out");
    for(int i = 0; i < 100; ++i) {
        REQUIRE_NOTHROW(outQ->get<dai::ImgFrame>());
    }
}

TEST_CASE("JPEG_ENCODING_LOSSLESS") {
    dai::Device device(getPipeline(dai::VideoEncoderProperties::Profile::MJPEG, 30, true, 0));

    auto outQ = device.getOutputQueue("out");
    for(int i = 0; i < 100; ++i) {
        auto encfrm = outQ->get<dai::EncodedFrame>();
        REQUIRE(encfrm->getProfile() == dai::EncodedFrame::Profile::JPEG);
        REQUIRE(encfrm->getLossless() == true);
        REQUIRE(encfrm->getQuality() == 30);
    }
}

TEST_CASE("JPEG_ENCODING_LOSSY") {
    dai::Device device(getPipeline(dai::VideoEncoderProperties::Profile::MJPEG, 30, false, 0));

    auto outQ = device.getOutputQueue("out");
    for(int i = 0; i < 100; ++i) {
        auto encfrm = outQ->get<dai::EncodedFrame>();
        REQUIRE(encfrm->getProfile() == dai::EncodedFrame::Profile::JPEG);
        REQUIRE(encfrm->getLossless() == false);
        REQUIRE(encfrm->getQuality() == 30);
    }
}

TEST_CASE("AVC_ENCODING") {
    dai::Device device(getPipeline(dai::VideoEncoderProperties::Profile::H264_HIGH, 30, false, 8500000));

    auto outQ = device.getOutputQueue("out");
    for(int i = 0; i < 100; ++i) {
        auto encfrm = outQ->get<dai::EncodedFrame>();
        REQUIRE(encfrm->getProfile() == dai::EncodedFrame::Profile::AVC);
        REQUIRE(encfrm->getLossless() == false);
        if(i % 30 == 0) REQUIRE(encfrm->getFrameType() == dai::EncodedFrame::FrameType::I);
        REQUIRE(encfrm->getQuality() == 30);
        REQUIRE(encfrm->getBitrate() == 8500000);
    }
}

TEST_CASE("HEVC_ENCODING") {
    dai::Device device(getPipeline(dai::VideoEncoderProperties::Profile::H265_MAIN, 30, false, 8500000));

    auto outQ = device.getOutputQueue("out");
    for(int i = 0; i < 100; ++i) {
        auto encfrm = outQ->get<dai::EncodedFrame>();
        REQUIRE(encfrm->getProfile() == dai::EncodedFrame::Profile::HEVC);
        REQUIRE(encfrm->getLossless() == false);
        if(i % 30 == 0) REQUIRE(encfrm->getFrameType() == dai::EncodedFrame::FrameType::I);
        REQUIRE(encfrm->getQuality() == 30);
        REQUIRE(encfrm->getBitrate() == 8500000);
    }
}

TEST_CASE("LINK_TO_BOTH") {
    dai::Pipeline pipeline;
    auto camNode = pipeline.create<dai::node::ColorCamera>();
    auto encNode = pipeline.create<dai::node::VideoEncoder>();
    auto xlinkOut1 = pipeline.create<dai::node::XLinkOut>();
    auto xlinkOut2 = pipeline.create<dai::node::XLinkOut>();
    camNode->video.link(encNode->input);
    encNode->bitstream.link(xlinkOut1->input);
    encNode->out.link(xlinkOut2->input);

    camNode->setVideoSize(1280, 720);
    encNode->setProfile(dai::VideoEncoderProperties::Profile::H264_MAIN);
    xlinkOut1->setStreamName("out1");
    xlinkOut2->setStreamName("out2");

    REQUIRE_THROWS(dai::Device(pipeline));
}
