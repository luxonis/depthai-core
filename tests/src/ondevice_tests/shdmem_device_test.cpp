#include <catch2/catch_all.hpp>

#include <depthai/depthai.hpp>
#include <depthai/pipeline/InputQueue.hpp>
#include <depthai/utility/SharedMemory.hpp>

#if defined(__unix__) && !defined(__APPLE__)
#include <sys/mman.h>

#include "depthai/depthai.hpp"

TEST_CASE("Test Device Shared Memory FDs") {
    dai::Pipeline pipeline;

    auto script = pipeline.create<dai::node::Script>();

    script->setScript(R"(
        frame : Buffer = node.inputs['in'].get()

        node.outputs['out'].send(frame)
    )");

    auto inputQueue = script->inputs["in"].createInputQueue();
    auto outputQueue = script->outputs["out"].createOutputQueue();

    long fd = memfd_create("Test", 0);
    REQUIRE(fd > 0);
    
    const long size = 4096;
    ftruncate(fd, size);
    void *inFile = mmap(NULL, size, PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0);
    REQUIRE(inFile != nullptr);
    *(uint8_t*)inFile = 0xFF;
    munmap(inFile, size);

    auto inFrame = std::make_shared<dai::Buffer>();
    REQUIRE(inFrame != nullptr);

    inFrame->setData(fd);
    inFrame->data->setSize(size);

    auto inMemoryPtr = std::dynamic_pointer_cast<dai::SharedMemory>(inFrame->data);
    REQUIRE(inMemoryPtr != nullptr);
    REQUIRE(inMemoryPtr->getFd() > 0);
    
    pipeline.start();
    inputQueue->send(inFrame);

    // Retrieve the resized frame
    auto outFrame = outputQueue->get<dai::Buffer>();
    REQUIRE(outFrame != nullptr);

    REQUIRE(std::dynamic_pointer_cast<dai::VectorMemory>(outFrame->data) == nullptr);
    auto outMemoryPtr = std::dynamic_pointer_cast<dai::SharedMemory>(outFrame->data);
    REQUIRE(outMemoryPtr != nullptr);
    REQUIRE(outMemoryPtr->getFd() > 0);

    void *outFile = mmap(NULL, size, PROT_READ | PROT_WRITE, MAP_SHARED, outMemoryPtr->getFd(), 0);
    REQUIRE(outFile != nullptr);
    REQUIRE(*(uint8_t*)outFile == 0xFF);
    munmap(outFile, size);
}
#endif
