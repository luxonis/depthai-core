#include <catch2/catch_all.hpp>
#include <chrono>
#include <opencv2/core.hpp>
#include <vector>

#include "depthai/depthai.hpp"

TEST_CASE("VideoDecoder decodes VideoEncoder output", "[video-decoder]") {
    using Profile = dai::VideoEncoderProperties::Profile;
#ifdef DEPTHAI_HAVE_OPENH264_SUPPORT
    const auto profile = GENERATE(Profile::MJPEG, Profile::H264_BASELINE, Profile::H264_MAIN, Profile::H264_HIGH);
#else
    const auto profile = Profile::MJPEG;
#endif
    CAPTURE(profile);
    constexpr int NUM_FRAMES = 20;
    constexpr int NUM_INPUT_FRAMES = NUM_FRAMES + 4;
    dai::Pipeline pipeline;
    auto encoder = pipeline.create<dai::node::VideoEncoder>();
    encoder->setDefaultProfilePreset(10, profile);
    encoder->setNumBFrames(0);
    auto input = encoder->input.createInputQueue(NUM_INPUT_FRAMES, true);
    auto decoder = pipeline.create<dai::node::VideoDecoder>();
    encoder->out.link(decoder->input);
    auto output = decoder->out.createOutputQueue(NUM_INPUT_FRAMES, true);
    auto encodedOutput = encoder->out.createOutputQueue(NUM_INPUT_FRAMES, true);
    pipeline.start();
    std::vector<cv::Mat> expectedFrames;
    expectedFrames.reserve(NUM_INPUT_FRAMES);
    std::vector<std::shared_ptr<dai::ImgFrame>> frames;
    frames.reserve(NUM_INPUT_FRAMES);
    for(int i = 0; i < NUM_INPUT_FRAMES; ++i) {
        // Large, chroma-aligned tiles exercise color and spatial reconstruction without
        // depending on camera exposure. Change every frame to catch stale output.
        cv::Mat expected(400, 640, CV_8UC3);
        for(int row = 0; row < 2; ++row) {
            for(int col = 0; col < 4; ++col) {
                const int tile = row * 4 + col;
                expected(cv::Rect(col * 160, row * 200, 160, 200)) =
                    cv::Scalar(30 + (tile * 29 + i * 17) % 190, 30 + (tile * 53 + i * 31) % 190, 30 + (tile * 71 + i * 47) % 190);
            }
        }
        auto frame = std::make_shared<dai::ImgFrame>();
        frame->setCvFrame(expected, dai::ImgFrame::Type::NV12);
        frame->setSequenceNum(i);
        frame->setTimestamp(std::chrono::steady_clock::now());
        frames.push_back(frame);
        expectedFrames.push_back(expected);
    }
    // H.264 needs extra input to release buffered frames; JPEG can be sent on demand.
    if(profile != Profile::MJPEG) {
        for(const auto& frame : frames) input->send(frame);
    }
    for(int i = 0; i < NUM_FRAMES; ++i) {
        CAPTURE(i);
        if(profile == Profile::MJPEG) input->send(frames.at(i));
        bool timedOut = false;
        auto decoded = output->get<dai::ImgFrame>(std::chrono::seconds(10), timedOut);
        REQUIRE_FALSE(timedOut);
        REQUIRE(decoded);
        auto encoded = encodedOutput->get<dai::EncodedFrame>(std::chrono::seconds(10), timedOut);
        REQUIRE_FALSE(timedOut);
        REQUIRE(encoded);
        REQUIRE(decoded->getSequenceNum() == i);
        REQUIRE(decoded->getWidth() == 640);
        REQUIRE(decoded->getHeight() == 400);
        REQUIRE(decoded->getType() == dai::ImgFrame::Type::BGR888i);
        REQUIRE(decoded->getData().size() == 640 * 400 * 3);
        REQUIRE(decoded->getSequenceNum() == encoded->getSequenceNum());
        REQUIRE(decoded->getTimestamp() == encoded->getTimestamp());
        REQUIRE(decoded->getTimestampDevice() == encoded->getTimestampDevice());
        // Allow compression and JPEG/H.264 YUV range differences: less than 5% of the 8-bit range.
        const auto decodedImage = decoded->getCvFrame();
        cv::Mat difference;
        cv::absdiff(decodedImage, expectedFrames.at(i), difference);
        const auto meanError = cv::mean(difference);
        CAPTURE(meanError);
        for(int channel = 0; channel < 3; ++channel) {
            REQUIRE(meanError[channel] < 12.0);
        }
    }
    pipeline.stop();
    pipeline.wait();
}
