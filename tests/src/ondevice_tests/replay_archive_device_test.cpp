#include <catch2/catch_all.hpp>
#include <chrono>
#include <iostream>

#include "../ReplayImageArchiveFixture.hpp"
#include "depthai/depthai.hpp"

namespace {

std::shared_ptr<dai::Device> openPlatform(XLinkPlatform_t platform) {
    for(const auto& info : dai::Device::getAllAvailableDevices()) {
        if(info.platform == platform) {
            std::cout << "Opening archive test device " << info.getDeviceId() << '\n';
            return std::make_shared<dai::Device>(info);
        }
    }
    return nullptr;
}

void checkFrames(const std::shared_ptr<dai::MessageQueue>& queue, int width, int height, dai::ImgFrame::Type type, int count) {
    std::vector<int> luma;
    for(int index = 0; index < count; ++index) {
        bool timeout = false;
        const auto frame = queue->get<dai::ImgFrame>(std::chrono::seconds(5), timeout);
        REQUIRE_FALSE(timeout);
        REQUIRE(frame);
        REQUIRE(frame->getWidth() == width);
        REQUIRE(frame->getHeight() == height);
        REQUIRE(frame->getType() == type);
        REQUIRE(frame->getSequenceNum() == index);
        REQUIRE_FALSE(frame->getData().empty());
        luma.push_back(frame->getData()[0]);
    }
    REQUIRE(luma[1] > luma[0] + 20);
    REQUIRE(luma[2] > luma[1] + 20);
    if(count == 6) {
        REQUIRE(luma[3] == luma[0]);
        REQUIRE(luma[4] == luma[1]);
        REQUIRE(luma[5] == luma[2]);
    }
}

}  // namespace

TEST_CASE("Host archive replay feeds the RVC2 ImageManip", "[archive][rvc2]") {
    const auto extension = GENERATE(".jpg", ".png");
    const auto mode = GENERATE(dai::ReplayDecodeMode::STREAM, dai::ReplayDecodeMode::PRE_DECODE);
    auto device = openPlatform(X_LINK_MYRIAD_X);
    if(!device) SKIP("No RVC2 connected");
    replaytest::ImageArchive source(extension, ".tar.gz", true, 64, 48);
    dai::Pipeline pipeline(device);
    auto replay = pipeline.create<dai::node::ReplayVideo>();
    replay->setSource(source.archive).setRunOnHost(true).setLoop(false).setStopPipelineAtEnd(false).setFps(10.f).setDecodeMode(mode);
    replay->setOutFrameType(dai::ImgFrame::Type::NV12);
    auto manip = pipeline.create<dai::node::ImageManip>();
    manip->initialConfig->setOutputSize(32, 24);
    manip->initialConfig->setFrameType(dai::ImgFrame::Type::GRAY8);
    manip->setMaxOutputFrameSize(32 * 24);
    replay->out.link(manip->inputImage);
    auto queue = manip->out.createOutputQueue(8, true);
    pipeline.start();
    REQUIRE(replay->runOnHost());
    REQUIRE_FALSE(manip->runOnHost());
    checkFrames(queue, 32, 24, dai::ImgFrame::Type::GRAY8, 3);
    pipeline.stop();
}

TEST_CASE("RVC4 archive replay decodes JPEG and PNG members", "[archive][rvc4]") {
    const auto extension = GENERATE(".jpg", ".png");
    const auto archiveExtension = GENERATE(".tar", ".tar.gz", ".tgz", ".tar.xz");
    const auto mode = GENERATE(dai::ReplayDecodeMode::STREAM, dai::ReplayDecodeMode::PRE_DECODE);
    auto device = openPlatform(X_LINK_RVC4);
    if(!device) SKIP("No RVC4 connected");
    replaytest::ImageArchive source(extension, archiveExtension, true);
    dai::Pipeline pipeline(device);
    auto replay = pipeline.create<dai::node::ReplayVideo>();
    replay->setSource(source.archive).setRunOnHost(false).setLoop(true).setStopPipelineAtEnd(false).setFps(30.f).setDecodeMode(mode);
    replay->setOutFrameType(dai::ImgFrame::Type::NV12);
    auto queue = replay->out.createOutputQueue(8, true);
    pipeline.start();
    checkFrames(queue, 18, 10, dai::ImgFrame::Type::NV12, 6);
    pipeline.stop();
}

TEST_CASE("RVC4 predecode keeps archive streams separate", "[archive][rvc4]") {
    auto device = openPlatform(X_LINK_RVC4);
    if(!device) SKIP("No RVC4 connected");
    replaytest::Directory temporary;
    const auto archive = temporary.path / "streams.tar";
    std::vector<replaytest::Member> members{{"./", {}, true}, {"./left/", {}, true}, {"./right/", {}, true}};
    for(const auto& stream : {std::string("left"), std::string("right")}) {
        for(int index = 0; index < 2; ++index) {
            const int value = stream == "left" ? 40 : 180;
            const cv::Mat image(10, 18, CV_8UC3, cv::Scalar(value, value, value));
            std::vector<std::uint8_t> bytes;
            REQUIRE(cv::imencode(".png", image, bytes));
            members.push_back({"./" + stream + "/cam_" + std::to_string(index) + ".png", std::move(bytes)});
        }
    }
    replaytest::writeArchive(archive, members);
    dai::Pipeline pipeline(device);
    auto left = pipeline.create<dai::node::ReplayVideo>();
    auto right = pipeline.create<dai::node::ReplayVideo>();
    for(const auto& replay : {left, right}) {
        replay->setSource(archive).setRunOnHost(false).setLoop(false).setStopPipelineAtEnd(false);
        replay->setDecodeMode(dai::ReplayDecodeMode::PRE_DECODE).setOutFrameType(dai::ImgFrame::Type::NV12);
    }
    left->setArchiveEntry("left");
    right->setArchiveEntry("right");
    auto leftQueue = left->out.createOutputQueue(8, true);
    auto rightQueue = right->out.createOutputQueue(8, true);
    pipeline.start();
    for(int index = 0; index < 2; ++index) {
        bool timeout = false;
        const auto leftFrame = leftQueue->get<dai::ImgFrame>(std::chrono::seconds(5), timeout);
        REQUIRE_FALSE(timeout);
        REQUIRE(leftFrame);
        const auto rightFrame = rightQueue->get<dai::ImgFrame>(std::chrono::seconds(5), timeout);
        REQUIRE_FALSE(timeout);
        REQUIRE(rightFrame);
        REQUIRE(rightFrame->getData()[0] > leftFrame->getData()[0] + 80);
    }
    pipeline.stop();
}

TEST_CASE("RVC4 archive replay resizes odd and one-pixel image sources", "[archive][rvc4]") {
    const auto dimensions = GENERATE(std::make_pair(17, 9), std::make_pair(1, 1));
    const auto mode = GENERATE(dai::ReplayDecodeMode::STREAM, dai::ReplayDecodeMode::PRE_DECODE);
    auto device = openPlatform(X_LINK_RVC4);
    if(!device) SKIP("No RVC4 connected");
    replaytest::ImageArchive source(".png", ".tar.gz", true, dimensions.first, dimensions.second);
    dai::Pipeline pipeline(device);
    auto replay = pipeline.create<dai::node::ReplayVideo>();
    replay->setSource(source.archive).setRunOnHost(false).setLoop(false).setStopPipelineAtEnd(false).setDecodeMode(mode);
    replay->setOutFrameType(dai::ImgFrame::Type::NV12).setSize(16, 8);
    auto queue = replay->out.createOutputQueue(8, true);
    pipeline.start();
    checkFrames(queue, 16, 8, dai::ImgFrame::Type::NV12, 3);
    pipeline.stop();
}
