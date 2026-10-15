#include <catch2/catch_all.hpp>
#include <chrono>
#include <opencv2/imgcodecs.hpp>
#include <thread>

#include "depthai/depthai.hpp"

namespace {
std::shared_ptr<dai::EncodedFrame> makeFrame(int width = 32, int height = 24, bool padded = false) {
    cv::Mat image(height, width, CV_8UC3, cv::Scalar(30, 90, 180));
    std::vector<uint8_t> jpeg;
    REQUIRE(cv::imencode(".jpg", image, jpeg));
    auto frame = std::make_shared<dai::EncodedFrame>();
    frame->setProfile(dai::EncodedFrame::Profile::JPEG);
    frame->setSize(width, height);
    frame->transformation = dai::ImgTransformation(width, height);
    frame->setSequenceNum(42);
    frame->setTimestamp(std::chrono::steady_clock::now());
    frame->setTimestampSystem(std::chrono::system_clock::now());
    frame->setTimestampDevice(std::chrono::steady_clock::time_point(std::chrono::seconds(10)));
    frame->setInstanceNum(3);
    frame->cam.sensitivityIso = 200;
    if(padded) {
        frame->frameOffset = 7;
        frame->frameSize = static_cast<uint32_t>(jpeg.size());
        jpeg.insert(jpeg.begin(), 7, 0);
        jpeg.insert(jpeg.end(), 5, 0);
    }
    frame->setData(jpeg);
    return frame;
}
}  // namespace

TEST_CASE("VideoDecoder decodes MJPEG and converts output at runtime", "[video-decoder]") {
    const bool padded = GENERATE(false, true);
    dai::Pipeline pipeline(false);
    auto decoder = pipeline.create<dai::node::VideoDecoder>();
    REQUIRE(decoder->getOutFrameType() == dai::ImgFrame::Type::BGR888i);
    REQUIRE_THROWS_AS(decoder->setOutFrameType(dai::ImgFrame::Type::BITSTREAM), std::invalid_argument);
    auto input = decoder->input.createInputQueue();
    auto output = decoder->out.createOutputQueue();
    pipeline.start();
    using Type = dai::ImgFrame::Type;
    for(auto type : {Type::BGR888i, Type::RGB888i, Type::BGR888p, Type::RGB888p, Type::GRAY8, Type::RAW8, Type::YUV420p, Type::NV12, Type::NV21}) {
        CAPTURE(type);
        decoder->setOutFrameType(type);
        auto encoded = makeFrame(32, 24, padded);
        input->send(encoded);
        bool timedOut = false;
        auto decoded = output->get<dai::ImgFrame>(std::chrono::seconds(5), timedOut);
        REQUIRE_FALSE(timedOut);
        REQUIRE(decoded);
        REQUIRE(decoded->getType() == type);
        REQUIRE(decoded->getWidth() == 32);
        REQUIRE(decoded->getHeight() == 24);
        REQUIRE(decoded->getSequenceNum() == encoded->getSequenceNum());
        REQUIRE(decoded->getTimestamp() == encoded->getTimestamp());
        REQUIRE(decoded->getTimestampSystem() == encoded->getTimestampSystem());
        REQUIRE(decoded->getTimestampDevice() == encoded->getTimestampDevice());
        REQUIRE(decoded->getInstanceNum() == 3);
        REQUIRE(decoded->cam.sensitivityIso == 200);
        REQUIRE(decoded->transformation.getSourceSize() == encoded->transformation.getSourceSize());
        REQUIRE(decoded->transformation.getTransformationMatrix() == encoded->transformation.getTransformationMatrix());
        auto image = decoded->getCvFrame();
        const auto mean = cv::mean(image);
        if(type == Type::GRAY8 || type == Type::RAW8) {
            REQUIRE(image.channels() == 1);
            REQUIRE(std::abs(mean[0] - 110) < 4);
        } else {
            REQUIRE(image.channels() == 3);
            REQUIRE(std::abs(mean[0] - 30) < 5);
            REQUIRE(std::abs(mean[1] - 90) < 5);
            REQUIRE(std::abs(mean[2] - 180) < 5);
        }
    }
    pipeline.stop();
    pipeline.wait();
}

TEST_CASE("VideoDecoder rejects invalid frames without terminating the process", "[video-decoder]") {
    dai::Pipeline pipeline(false);
    auto decoder = pipeline.create<dai::node::VideoDecoder>();
    auto input = decoder->input.createInputQueue();
    auto output = decoder->out.createOutputQueue();
    auto frame = makeFrame();
    SECTION("H264") {
        frame->setProfile(dai::EncodedFrame::Profile::AVC);
    }
    SECTION("H265") {
        frame->setProfile(dai::EncodedFrame::Profile::HEVC);
    }
    SECTION("empty") {
        frame->setData(std::vector<uint8_t>{});
    }
    SECTION("invalid offset") {
        frame->frameOffset = static_cast<uint32_t>(frame->getData().size() + 1);
    }
    SECTION("invalid size") {
        frame->frameSize = static_cast<uint32_t>(frame->getData().size() + 1);
    }
    SECTION("invalid JPEG") {
        frame->setData(std::vector<uint8_t>{0xff, 0xd8, 0, 0});
    }
    SECTION("wrong image format") {
        frame->setData(std::vector<uint8_t>{0x89, 0x50, 0x4e, 0x47});
    }
    SECTION("odd YUV dimensions") {
        frame = makeFrame(31, 23);
        decoder->setOutFrameType(dai::ImgFrame::Type::NV12);
    }
    pipeline.start();
    input->send(frame);
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(5);
    while(decoder->isRunning() && std::chrono::steady_clock::now() < deadline) std::this_thread::sleep_for(std::chrono::milliseconds(10));
    REQUIRE_FALSE(decoder->isRunning());
    pipeline.stop();
    pipeline.wait();
    REQUIRE_FALSE(pipeline.isRunning());
}
