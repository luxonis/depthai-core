#include <catch2/catch_all.hpp>
#include <chrono>
#include <map>

#include "depthai/depthai.hpp"

TEST_CASE("VideoDecoder decodes VideoEncoder output", "[video-decoder]") {
    using Profile = dai::VideoEncoderProperties::Profile;
#ifdef DEPTHAI_HAVE_OPENH264_SUPPORT
    const auto profile = GENERATE(Profile::MJPEG, Profile::H264_BASELINE, Profile::H264_MAIN, Profile::H264_HIGH);
#else
    const auto profile = Profile::MJPEG;
#endif
    CAPTURE(profile);
    dai::Pipeline pipeline;
    auto camera = pipeline.create<dai::node::Camera>()->build();
    auto encoder = pipeline.create<dai::node::VideoEncoder>();
    encoder->setDefaultProfilePreset(10, profile);
    encoder->setNumBFrames(0);
    camera->requestOutput({640, 400}, dai::ImgFrame::Type::NV12, dai::ImgResizeMode::CROP, 10)->link(encoder->input);
    auto decoder = pipeline.create<dai::node::VideoDecoder>();
    encoder->out.link(decoder->input);
    auto output = decoder->out.createOutputQueue();
    auto encodedOutput = encoder->out.createOutputQueue();
    pipeline.start();
    std::map<int64_t, std::shared_ptr<dai::EncodedFrame>> metadata;
    for(int i = 0; i < 20; ++i) {
        bool timedOut = false;
        auto decoded = output->get<dai::ImgFrame>(std::chrono::seconds(10), timedOut);
        REQUIRE_FALSE(timedOut);
        REQUIRE(decoded);
        while(metadata.find(decoded->getSequenceNum()) == metadata.end()) {
            auto encoded = encodedOutput->get<dai::EncodedFrame>(std::chrono::seconds(10), timedOut);
            REQUIRE_FALSE(timedOut);
            REQUIRE(encoded);
            metadata.emplace(encoded->getSequenceNum(), encoded);
            REQUIRE(metadata.size() < 64);
        }
        auto encoded = metadata.at(decoded->getSequenceNum());
        metadata.erase(decoded->getSequenceNum());
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
