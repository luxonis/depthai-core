#include <catch2/catch_all.hpp>
#include <chrono>
#include <thread>

#include "depthai/depthai.hpp"
#include "depthai/pipeline/InputQueue.hpp"

TEST_CASE("Test ColorCamera node") {
    using namespace std;
    using namespace std::chrono;
    using namespace std::chrono_literals;

    dai::Pipeline p;
    auto imageManipNode = p.create<dai::node::ImageManip>();

    auto inputQueue = imageManipNode->inputImage.createInputQueue();
    auto outputQueue = imageManipNode->out.createOutputQueue();

    int inputWidth, inputHeight;
    int outputWidth, outputHeight;
    SECTION("640x320 -> 320x320") {
        inputWidth = 640, inputHeight = 320;
        outputWidth = 320, outputHeight = 320;
    }

    SECTION("1280x720 -> 200x145") {
        inputWidth = 1280, inputHeight = 720;
        outputWidth = 200, outputHeight = 145;
    }

    imageManipNode->initialConfig.setResize(outputWidth, outputHeight);

    p.start();

    // Retrieve N messages
    constexpr size_t N = 20;
    for(size_t i = 0; i < N; ++i) {
        auto inFrame = std::make_shared<dai::ImgFrame>();
        inFrame->setData(std::vector<uint8_t>(inputWidth * inputHeight * 3));
        inFrame->setWidth(inputWidth);
        inFrame->setHeight(inputHeight);
        inFrame->setType(dai::ImgFrame::Type::RGB888p);

        inputQueue->send(inFrame);

        // Retrieve the resized frame
        auto outFrame = outputQueue->get<dai::ImgFrame>();

        REQUIRE(outFrame->getWidth() == outputWidth);
        REQUIRE(outFrame->getHeight() == outputHeight);
    }
}
