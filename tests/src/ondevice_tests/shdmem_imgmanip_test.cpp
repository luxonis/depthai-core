#include <catch2/catch_all.hpp>

#include <depthai/depthai.hpp>
#include <depthai/pipeline/InputQueue.hpp>
#include <depthai/utility/SharedMemory.hpp>

#if defined(__unix__) && !defined(__APPLE__)
#include <sys/mman.h>

#include "depthai/depthai.hpp"

TEST_CASE("Test Device Shared Memory FDs ImgManip") {
    dai::Pipeline pipeline;

    auto manip = pipeline.create<dai::node::ImageManip>();

    auto inputQueue = manip->inputImage.createInputQueue();
    auto outputQueue = manip->out.createOutputQueue();

    long fd = memfd_create("Test", 0);
    REQUIRE(fd > 0);

    const long size = 1024 * 768 * 2;
    ftruncate(fd, size);
    void *inFile = mmap(NULL, size, PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0);
    REQUIRE(inFile != nullptr);
    memset(inFile, 0xFF, size);
    munmap(inFile, size);

    auto inFrame = std::make_shared<dai::ImgFrame>();
    REQUIRE(inFrame != nullptr);
    inFrame->setData(fd);
    inFrame->data->setSize(size);
    inFrame->setWidth(1024);
    inFrame->setHeight(768);
    inFrame->setType(dai::ImgFrame::Type::NV12);

    ftruncate(fd, size);

    auto inMemoryPtr = std::dynamic_pointer_cast<dai::SharedMemory>(inFrame->data);
    REQUIRE(inMemoryPtr != nullptr);
    REQUIRE(inMemoryPtr->getFd() > 0);

    manip->initialConfig.setResize(640, 480);
    manip->initialConfig.setFrameType(dai::ImgFrame::Type::NV12);

    pipeline.start();
    inputQueue->send(inFrame);

    // Retrieve the resized frame
    auto outFrame = outputQueue->get<dai::ImgFrame>();
    REQUIRE(outFrame != nullptr);

    auto outMemoryPtr = std::dynamic_pointer_cast<dai::SharedMemory>(outFrame->data);
    REQUIRE(outMemoryPtr != nullptr);
    REQUIRE(outMemoryPtr->getFd() > 0);

    void *outFile = mmap(NULL, size, PROT_READ | PROT_WRITE, MAP_SHARED, outMemoryPtr->getFd(), 0);
    REQUIRE(outFile != nullptr);
    REQUIRE(*(uint8_t*)outFile == 0xFF);
    munmap(outFile, size);
}
/*
TEST_CASE("Test Device Shared Memory FDs ImgManip Change Format") {
    dai::Pipeline pipeline;

    auto manip = pipeline.create<dai::node::ImageManip>();

    auto inputQueue = manip->inputImage.createInputQueue();
    auto outputQueue = manip->out.createOutputQueue();

    long fd = memfd_create("Test", 0);
    REQUIRE(fd > 0);

    long size = 1024 * 768 * 3;
    auto inFrame = std::make_shared<dai::ImgFrame>();
    REQUIRE(inFrame != nullptr);
    inFrame->setData(fd);
    inFrame->data->setSize(size);
    inFrame->setWidth(1024);
    inFrame->setHeight(768);
    inFrame->setType(dai::ImgFrame::Type::NV12);

    ftruncate(fd, size);

    auto inMemoryPtr = std::dynamic_pointer_cast<dai::SharedMemory>(inFrame->data);
    REQUIRE(inMemoryPtr != nullptr);
    REQUIRE(inMemoryPtr->getFd() > 0);

    manip->initialConfig.setResize(300, 300);
    manip->initialConfig.setFrameType(dai::ImgFrame::Type::BGR888p);
    
    pipeline.start();
    inputQueue->send(inFrame);

    // Retrieve the resized frame
    auto outFrame = outputQueue->get<dai::ImgFrame>();
    REQUIRE(outFrame != nullptr);

    auto outMemoryPtr = std::dynamic_pointer_cast<dai::SharedMemory>(outFrame->data);
    REQUIRE(outMemoryPtr != nullptr);
    REQUIRE(outMemoryPtr->getFd() > 0);
}*/

#endif
