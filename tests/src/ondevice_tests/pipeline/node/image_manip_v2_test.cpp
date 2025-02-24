#include <catch2/catch_all.hpp>
#include <catch2/catch_test_macros.hpp>

#include "depthai/common/CameraBoardSocket.hpp"
#include "depthai/depthai.hpp"
#include "depthai/pipeline/datatype/ImgFrame.hpp"
#include "depthai/pipeline/node/Camera.hpp"

void testManipBasic(bool runSyncOnHost) {
    // Create pipeline
    dai::Pipeline p;
    auto cam = p.create<dai::node::Camera>()->build(dai::CameraBoardSocket::CAM_A);
    auto manip = p.create<dai::node::ImageManipV2>();
    cam->requestFullResolutionOutput()->link(manip->inputImage);
    manip->initialConfig.addCrop(100, 200, 400, 400);
    manip->initialConfig.setFrameType(dai::ImgFrame::Type::NV12);

    auto manipQueue = manip->out.createOutputQueue();
    p.start();

    for(int i = 0; i < 10; i++) {
        auto inFrame = manipQueue->get<dai::ImgFrame>();
        REQUIRE(inFrame != nullptr);
        REQUIRE(inFrame->getWidth() == 400);
        REQUIRE(inFrame->getHeight() == 400);
        REQUIRE(inFrame->getType() == dai::ImgFrame::Type::NV12);
    }
}

void testManipDynamic(bool runSyncOnHost, bool reuiseImage) {
    // Create pipeline
    dai::Pipeline p;
    auto cam = p.create<dai::node::Camera>()->build(dai::CameraBoardSocket::CAM_A);
    auto manip = p.create<dai::node::ImageManipV2>();
    cam->requestFullResolutionOutput()->link(manip->inputImage);
    auto manipQueue = manip->out.createOutputQueue();
    manip->inputConfig.setReusePreviousMessage(false);  // Control the rate with the config input
    auto configQueue = manip->inputConfig.createInputQueue();
    p.start();
    int cropX = 100;
    int cropY = 200;
    int cropW = 400;
    int cropH = 400;
    for(int i = 0; i < 10; i++) {
        cropX = cropX + 10;
        cropY = cropY + 10;
        cropW = cropW + 10;
        cropH = cropH + 10;
        auto manipConfig = std::make_shared<dai::ImageManipConfigV2>();
        manipConfig->addCrop(cropX, cropY, cropW, cropH);
        manipConfig->setFrameType(dai::ImgFrame::Type::NV12);
        manipConfig->setReusePreviousImage(reuiseImage);
        configQueue->send(manipConfig);
        auto inFrame = manipQueue->get<dai::ImgFrame>();
        REQUIRE(inFrame != nullptr);
        REQUIRE(inFrame->getType() == dai::ImgFrame::Type::NV12);
        REQUIRE(inFrame->getWidth() == cropW);
        REQUIRE(inFrame->getHeight() == cropH);
    }
}

TEST_CASE("Sync node runs on device") {
    testManipBasic(false);
}

TEST_CASE("Sync node runs on host") {
    testManipBasic(true);
}

TEST_CASE("Dynamic config sync node runs on device, reuse") {
    testManipDynamic(false, true);
}

TEST_CASE("Dynamic config sync node runs on host, reuse") {
    testManipDynamic(true, true);
}

TEST_CASE("Dynamic config sync node runs on device, no reuse") {
    testManipDynamic(false, false);
}

TEST_CASE("Dynamic config sync node runs on host, no reuse") {
    testManipDynamic(true, false);
}

TEST_CASE("Test ImageManipV2 with u16 frames") {
    using namespace std;
    using namespace std::chrono;
    using namespace std::chrono_literals;

    constexpr int inputWidth = 640, inputHeight = 480;
    constexpr int outputWidth = 320, outputHeight = 320;
    constexpr size_t N = 20;

    dai::Pipeline p;
    auto manip = p.create<dai::node::ImageManipV2>();
    manip->initialConfig.setOutputSize(outputWidth, outputHeight);

    auto inputQueue = manip->inputImage.createInputQueue();
    auto outputQueue = manip->out.createOutputQueue();

    p.start();

    for(size_t i = 0; i < N; ++i) {
        auto inFrame = std::make_shared<dai::ImgFrame>();
        inFrame->setData(std::vector<uint8_t>(inputWidth * inputHeight * 2));
        inFrame->setWidth(inputWidth);
        inFrame->setHeight(inputHeight);
        inFrame->setStride(inputWidth * 2);
        inFrame->setType(dai::ImgFrame::Type::RAW16);

        inputQueue->send(inFrame);

        // Retrieve the resized frame
        auto outFrame = outputQueue->get<dai::ImgFrame>();

        REQUIRE(outFrame->getWidth() == outputWidth);
        REQUIRE(outFrame->getHeight() == outputHeight);
    }
}

TEST_CASE("ImageManipV2 rebuild on cfg change") {
    dai::Pipeline p;
    auto cam = p.create<dai::node::Camera>()->build(dai::CameraBoardSocket::CAM_A);
    auto manip = p.create<dai::node::ImageManipV2>();
    manip->setRunOnHost(true);
    cam->requestFullResolutionOutput()->link(manip->inputImage);
    manip->initialConfig.setOutputSize(400, 200);
    manip->inputConfig.setWaitForMessage(true);

    auto manipQueue = manip->out.createOutputQueue();
    auto icQueue = manip->inputConfig.createInputQueue();
    p.start();
    auto imgFrame = manipQueue->get<dai::ImgFrame>();
    REQUIRE(imgFrame->getWidth() == 400);
    REQUIRE(imgFrame->getHeight() == 200);
    dai::ImageManipConfigV2 cfg;
    cfg.setOutputSize(200, 400);
    icQueue->send(std::make_shared<dai::ImageManipConfigV2>(cfg));
    imgFrame = manipQueue->get<dai::ImgFrame>();
    REQUIRE(imgFrame->getWidth() == 200);
    REQUIRE(imgFrame->getHeight() == 400);
    p.stop();
}
