#include <catch2/catch_all.hpp>
#include <catch2/catch_test_macros.hpp>
#include <chrono>
#include <depthai/depthai.hpp>
#include <depthai/pipeline/node/internal/XLinkIn.hpp>
#include <depthai/pipeline/node/internal/XLinkOut.hpp>
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

TEST_CASE("XLinkOut PacketSize") {
    dai::Pipeline p;
    auto camera = p.create<dai::node::Camera>()->build();
    auto* cameraOutput = camera->requestFullResolutionOutput();

    auto xLinkOut = p.create<dai::node::internal::XLinkOut>();

    // xLinkOut->setPacketSize(1024);

    // cameraOutput->link(xLinkOut->input);

    auto out = cameraOutput->createOutputQueue();

    p.start();
    for(int i = 0; i < 10; ++i) {
        std::cout << "1234\n";
        // auto frame = out->get<dai::ImgFrame>();
    }
}
