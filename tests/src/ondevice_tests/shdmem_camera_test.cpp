#include <catch2/catch_all.hpp>

#include <depthai/depthai.hpp>
#include <depthai/pipeline/InputQueue.hpp>
#include <depthai/utility/SharedMemory.hpp>

#if defined(__unix__) && !defined(__APPLE__)
#include <sys/mman.h>

#include "depthai/depthai.hpp"

TEST_CASE("Test Device Shared Memory FDs Camera") {
    dai::Pipeline pipeline;

    auto cam = pipeline.create<dai::node::Camera>();
    cam->setBoardSocket(dai::CameraBoardSocket::CAM_A);  // TODO(Morato) - change to semantic name

    dai::ImgFrameCapability cap;
    cap.type = dai::ImgFrame::Type::NV12;  // Fastest
    cap.size.fixed(std::pair<int, int>(1280, 800));
    auto outputQueue = cam->requestOutput(cap, true)->createOutputQueue();

    pipeline.start();

    auto videoIn = outputQueue->get<dai::ImgFrame>();
    REQUIRE(videoIn != nullptr);

    auto memoryPtr = std::dynamic_pointer_cast<dai::SharedMemory>(videoIn->data);
    REQUIRE(memoryPtr != nullptr);
    REQUIRE(memoryPtr->getFd() > 0);
}

#endif
