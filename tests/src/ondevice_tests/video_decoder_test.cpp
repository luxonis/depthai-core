#include <catch2/catch_all.hpp>
#include <chrono>

#include "depthai/depthai.hpp"

TEST_CASE("VideoDecoder decodes VideoEncoder MJPEG output", "[video-decoder]") {
    dai::Pipeline pipeline;
    auto camera = pipeline.create<dai::node::Camera>()->build();
    auto encoder = pipeline.create<dai::node::VideoEncoder>();
    encoder->setDefaultProfilePreset(10, dai::VideoEncoderProperties::Profile::MJPEG);
    camera->requestOutput({640, 400}, dai::ImgFrame::Type::NV12, dai::ImgResizeMode::CROP, 10)->link(encoder->input);
    auto decoder = pipeline.create<dai::node::VideoDecoder>();
    encoder->out.link(decoder->input);
    auto output = decoder->out.createOutputQueue();
    auto encodedOutput = encoder->out.createOutputQueue();
    pipeline.start();
    for(int i = 0; i < 10; ++i) {
        bool timedOut = false;
        auto encoded = encodedOutput->get<dai::EncodedFrame>(std::chrono::seconds(10), timedOut);
        REQUIRE_FALSE(timedOut);
        REQUIRE(encoded);
        auto decoded = output->get<dai::ImgFrame>(std::chrono::seconds(10), timedOut);
        REQUIRE_FALSE(timedOut);
        REQUIRE(decoded);
        REQUIRE(decoded->getWidth() == 640);
        REQUIRE(decoded->getHeight() == 400);
        REQUIRE(decoded->getType() == dai::ImgFrame::Type::BGR888i);
        REQUIRE(decoded->getData().size() == 640 * 400 * 3);
        REQUIRE(decoded->getSequenceNum() == encoded->getSequenceNum());
        REQUIRE(decoded->getTimestamp() == encoded->getTimestamp());
        REQUIRE(decoded->getTimestampDevice() == encoded->getTimestampDevice());
        REQUIRE(cv::mean(decoded->getCvFrame())[0] > 0);
    }
    pipeline.stop();
    pipeline.wait();
}
