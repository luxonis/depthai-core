#include <catch2/catch_test_macros.hpp>
#include <chrono>

#include "../utility/DeblurFixture.hpp"
#include "depthai/depthai.hpp"

TEST_CASE("RVC4 Deblur CPU and GPU match the reference with exact pairing", "[Deblur][RVC4]") {
    for(const bool gpu : {false, true}) {
        auto device = std::make_shared<dai::Device>();
        REQUIRE(device->getPlatform() == dai::Platform::RVC4);
        dai::Pipeline pipeline(device);
        auto node = pipeline.create<dai::node::Deblur>();
        auto config = deblur_test::config();
        config.useOpenCL = gpu;
        config.maxWaitMs = 500;
        node->setConfig(config);
        auto images = node->inputImage.createInputQueue(4, true), flows = node->inputFlow.createInputQueue(2, true);
        auto output = node->output.createOutputQueue(2, true), passed = node->passthroughInputImage.createOutputQueue(2, true),
             masks = node->outputMask.createOutputQueue(2, true);
        pipeline.start();
        for(const auto size : std::vector<cv::Size>{{256, 128}, {640, 384}, {1280, 736}, {1920, 1056}}) {
            const auto fixture = deblur_test::make(size.width, size.height, size.width, 1000 + size.width);
            const auto field = dai::gate::deblur::prepare(*fixture.reference, *fixture.target, *fixture.packet, config);
            REQUIRE(field.kernels.size() == 2);
            const auto expected = dai::gate::deblur::finish(field, dai::gate::deblur::filterCpu(field), config);
            REQUIRE(images->trySend(fixture.reference));
            bool timedOut = false;
            auto initial = output->get<dai::ImgFrame>(std::chrono::seconds(20), timedOut);
            REQUIRE(!timedOut);
            REQUIRE(initial);
            REQUIRE(passed->get<dai::ImgFrame>(std::chrono::seconds(20), timedOut));
            REQUIRE(!timedOut);
            const auto firstMask = masks->get<dai::ImgFrame>(std::chrono::seconds(20), timedOut);
            REQUIRE(!timedOut);
            REQUIRE(firstMask);
            REQUIRE(std::all_of(firstMask->getData().begin(), firstMask->getData().end(), [](std::uint8_t v) { return v == 0; }));
            REQUIRE(flows->trySend(fixture.packet));
            REQUIRE(images->trySend(fixture.target));
            auto restored = output->get<dai::ImgFrame>(std::chrono::seconds(30), timedOut);
            REQUIRE(!timedOut);
            REQUIRE(restored);
            auto source = passed->get<dai::ImgFrame>(std::chrono::seconds(20), timedOut);
            REQUIRE(!timedOut);
            REQUIRE(source);
            auto mask = masks->get<dai::ImgFrame>(std::chrono::seconds(20), timedOut);
            REQUIRE(!timedOut);
            REQUIRE(mask);
            REQUIRE(restored->getSequenceNum() == fixture.target->getSequenceNum());
            REQUIRE(restored->getTimestampDevice() == fixture.target->getTimestampDevice());
            const auto actual = dai::gate::deblur::copyImage(*restored);
            INFO("GPU=" << gpu << " size=" << size.width << "x" << size.height);
            REQUIRE(cv::norm(actual, expected, cv::NORM_INF) <= 1);
            REQUIRE(cv::norm(actual, fixture.blurred, cv::NORM_INF) > 1);
            REQUIRE(cv::norm(dai::gate::deblur::copyImage(*source), fixture.blurred, cv::NORM_INF) == 0);
            for(std::size_t i = 0; i < mask->getData().size(); ++i)
                if(mask->getData()[i] == 0) REQUIRE(restored->getData()[i] == source->getData()[i]);
        }
        pipeline.stop();
    }
}
TEST_CASE("RVC4 OpticalFlow feeds Deblur without host flow export", "[Deblur][RVC4]") {
    auto device = std::make_shared<dai::Device>();
    dai::Pipeline pipeline(device);
    auto flow = pipeline.create<dai::node::OpticalFlow>();
    auto node = pipeline.create<dai::node::Deblur>();
    auto config = deblur_test::config();
    config.useOpenCL = true;
    config.maxWaitMs = 1000;
    node->setConfig(config);
    flow->passthroughInputImage.link(node->inputImage);
    flow->outputFlow.link(node->inputFlow);
    auto input = flow->inputImage.createInputQueue(2, true);
    auto output = node->output.createOutputQueue(2, true);
    auto masks = node->outputMask.createOutputQueue(2, true);
    const auto fixture = deblur_test::make(640, 384);
    pipeline.start();
    for(const auto& frame : {fixture.reference, fixture.target}) {
        REQUIRE(input->trySend(frame));
        bool timedOut = false;
        auto restored = output->get<dai::ImgFrame>(std::chrono::seconds(30), timedOut);
        REQUIRE(!timedOut);
        REQUIRE(restored);
        auto mask = masks->get<dai::ImgFrame>(std::chrono::seconds(20), timedOut);
        REQUIRE(!timedOut);
        REQUIRE(mask);
        REQUIRE(restored->getSequenceNum() == frame->getSequenceNum());
        if(frame == fixture.target) {
            REQUIRE(std::count_if(mask->getData().begin(), mask->getData().end(), [](std::uint8_t v) { return v > 0; }) > 100);
            REQUIRE(cv::norm(dai::gate::deblur::copyImage(*restored), fixture.blurred, cv::NORM_INF) > 0);
        }
    }
    pipeline.stop();
}

TEST_CASE("RVC4 Deblur shuts down when idle or backpressured", "[Deblur][RVC4]") {
    for(const bool backpressured : {false, true}) {
        auto device = std::make_shared<dai::Device>();
        dai::Pipeline pipeline(device);
        auto node = pipeline.create<dai::node::Deblur>();
        auto config = deblur_test::config();
        config.maxWaitMs = 0;
        node->setConfig(config);
        auto images = node->inputImage.createInputQueue(8, false);
        auto output = node->output.createOutputQueue(1, true);
        pipeline.start();
        if(backpressured) {
            const auto pixels = deblur_test::texture(256, 128);
            for(int i = 0; i < 8; ++i) images->send(deblur_test::image(pixels, i, 1000 + i));
            bool timedOut = false;
            REQUIRE(output->get<dai::ImgFrame>(std::chrono::seconds(20), timedOut));
            REQUIRE(!timedOut);
        }
        const auto start = std::chrono::steady_clock::now();
        pipeline.stop();
        REQUIRE(std::chrono::steady_clock::now() - start < std::chrono::seconds(10));
    }
}
TEST_CASE("RVC4 invalid Deblur input leaves an independent node alive", "[Deblur][RVC4]") {
    auto device = std::make_shared<dai::Device>();
    dai::Pipeline pipeline(device);
    auto bad = pipeline.create<dai::node::Deblur>(), good = pipeline.create<dai::node::Deblur>();
    auto config = deblur_test::config();
    config.maxWaitMs = 0;
    bad->setConfig(config);
    good->setConfig(config);
    auto badInput = bad->inputImage.createInputQueue(1, false), goodInput = good->inputImage.createInputQueue(1, true);
    auto output = good->output.createOutputQueue(1, true);
    pipeline.start();
    const auto pixels = deblur_test::texture(256, 128);
    auto invalid = deblur_test::image(pixels, 1, 1000);
    invalid->setType(dai::ImgFrame::Type::RAW8);
    REQUIRE(badInput->trySend(invalid));
    for(int i = 0; i < 3; ++i) {
        REQUIRE(goodInput->trySend(deblur_test::image(pixels, i + 10, 1040 + i * 40)));
        bool timedOut = false;
        const auto frame = output->get<dai::ImgFrame>(std::chrono::seconds(20), timedOut);
        REQUIRE(!timedOut);
        REQUIRE(frame);
        REQUIRE(frame->getSequenceNum() == i + 10);
        REQUIRE(cv::norm(dai::gate::deblur::copyImage(*frame), pixels, cv::NORM_INF) == 0);
    }
    pipeline.stop();
}

TEST_CASE("RVC4 Deblur history cannot exhaust live camera buffer pools", "[Deblur][RVC4]") {
    for(const auto size : std::vector<cv::Size>{{640, 384}, {1280, 736}, {1920, 1056}}) {
        auto device = std::make_shared<dai::Device>();
        dai::Pipeline pipeline(device);
        auto camera = pipeline.create<dai::node::Camera>()->build(size.width == 1920 ? dai::CameraBoardSocket::CAM_A : dai::CameraBoardSocket::CAM_B);
        auto flow = pipeline.create<dai::node::OpticalFlow>();
        auto node = pipeline.create<dai::node::Deblur>();
        auto config = node->getConfig();
        config.cacheSize = 8;
        node->setConfig(config);
        camera->requestOutput({size.width, size.height}, dai::ImgFrame::Type::GRAY8, dai::ImgResizeMode::CROP, 15)->link(flow->inputImage);
        flow->passthroughInputImage.link(node->inputImage);
        flow->outputFlow.link(node->inputFlow);
        auto output = node->output.createOutputQueue(2, true);
        pipeline.start();
        std::int64_t previous = -1;
        const int count = size.width == 640 ? 40 : 16;
        std::shared_ptr<dai::ImgFrame> retained;
        std::vector<std::uint8_t> snapshot;
        for(int i = 0; i < count; ++i) {
            bool timedOut = false;
            const auto frame = output->get<dai::ImgFrame>(std::chrono::seconds(20), timedOut);
            INFO("Live size=" << size.width << "x" << size.height << " frame=" << i);
            REQUIRE(!timedOut);
            REQUIRE(frame);
            REQUIRE(frame->getSequenceNum() > previous);
            REQUIRE(frame->getWidth() == size.width);
            REQUIRE(frame->getHeight() == size.height);
            previous = frame->getSequenceNum();
            if(!retained) {
                retained = frame;
                snapshot.assign(frame->getData().begin(), frame->getData().end());
            }
            REQUIRE(std::equal(snapshot.begin(), snapshot.end(), retained->getData().begin()));
        }
        pipeline.stop();
    }
}
