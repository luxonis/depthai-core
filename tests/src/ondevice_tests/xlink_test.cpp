#include <catch2/catch_all.hpp>
#include <catch2/catch_test_macros.hpp>
#include <chrono>
#include <depthai/depthai.hpp>
#include <thread>

using namespace std::chrono_literals;

TEST_CASE("XLinkIn lazy allocation test") {
    dai::Pipeline p;
    auto manip = p.create<dai::node::ImageManip>()->build();

    auto xLinkInImage = p.create<dai::node::internal::XLinkIn>();
    auto xLinkInConfig = p.create<dai::node::internal::XLinkIn>();

    xLinkInImage->setMaxDataSize(1024 * 1024 * 1024);  // 1GB per frame
    xLinkInImage->setNumFrames(64);

    xLinkInConfig->setMaxDataSize(1024 * 1024 * 1024);  // 1GB per frame
    xLinkInConfig->setNumFrames(64);

    xLinkInImage->out.link(manip->inputImage);
    xLinkInConfig->out.link(manip->inputConfig);

    auto run = [&]() {
        p.start();
        std::this_thread::sleep_for(1s);
    };

    // Without lazy allocation, this will throw as all the above XLinkIn nodes
    // would allocate all the frames at once.
    REQUIRE_NOTHROW(p.build());
    REQUIRE_NOTHROW(run());
}

TEST_CASE("XLinkBridge fps limit test") {
    constexpr float CAMERA_FPS = 30.0;
    constexpr float XLINK_FPS_LIMIT = 3.5;
    constexpr std::chrono::duration<double> TEST_DURATION = 2s;

    dai::Pipeline p;
    auto cam = p.create<dai::node::Camera>()->build();

    auto videoOutput = cam->requestOutput(std::make_pair(640, 400), std::nullopt, dai::ImgResizeMode::CROP, CAMERA_FPS);
    auto videoQueue = videoOutput->createOutputQueue();

    p.build();
    auto xlinkBridge = videoOutput->getXLinkBridge();
    REQUIRE(xlinkBridge != nullptr);
    REQUIRE(xlinkBridge->xLinkOut != nullptr);

    xlinkBridge->xLinkOut->setFpsLimit(XLINK_FPS_LIMIT);
    REQUIRE(xlinkBridge->xLinkOut->getFpsLimit() == XLINK_FPS_LIMIT);

    p.start();
    auto start = std::chrono::steady_clock::now();
    size_t numReceived = 0;
    while(p.isRunning()) {
        auto videoIn = videoQueue->get<dai::ImgFrame>();
        if(videoIn == nullptr) continue;
        numReceived++;

        if(std::chrono::steady_clock::now() - start > TEST_DURATION) {
            break;
        }
    }
    REQUIRE(numReceived == Catch::Approx(XLINK_FPS_LIMIT * TEST_DURATION.count()).margin(1.01));  // +- 1 frame
}