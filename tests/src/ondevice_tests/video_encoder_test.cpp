#include <catch2/catch_all.hpp>
#include <chrono>
#include <thread>

#include "depthai/depthai.hpp"

void testVideoEncoder(dai::VideoEncoderProperties::Profile profile) {
    using namespace std;
    using namespace std::chrono;
    using namespace std::chrono_literals;

    dai::Pipeline p;
    auto camRgb = p.create<dai::node::Camera>()->build(dai::CameraBoardSocket::CAM_A);
    auto camRgbOutput = camRgb->requestOutput({960, 720}, dai::ImgFrame::Type::NV12);

    auto encoder = p.create<dai::node::VideoEncoder>();
    camRgbOutput->link(encoder->input);

    encoder->setDefaultProfilePreset(30, profile);

    auto encoderOut = encoder->out.createOutputQueue();

    p.start();

    // Retrieve N messages
    constexpr size_t N = 20;
    for(size_t i = 0; i < N; ++i) {
        auto encoded = encoderOut->get<dai::EncodedFrame>();
        REQUIRE(encoded != nullptr);
    }
}

TEST_CASE("Test VideoEncoder node H264_BASELINE") {
    testVideoEncoder(dai::VideoEncoderProperties::Profile::H264_BASELINE);
}

TEST_CASE("Test VideoEncoder node H264_HIGH") {
    testVideoEncoder(dai::VideoEncoderProperties::Profile::H264_HIGH);
}

TEST_CASE("Test VideoEncoder node H264_MAIN") {
    testVideoEncoder(dai::VideoEncoderProperties::Profile::H264_MAIN);
}

TEST_CASE("Test VideoEncoder node H265_MAIN") {
    testVideoEncoder(dai::VideoEncoderProperties::Profile::H265_MAIN);
}

#if ENABLE_MJPEG != 0
TEST_CASE("Test VideoEncoder node MJPEG") {
    testVideoEncoder(dai::VideoEncoderProperties::Profile::MJPEG);
}
#endif
