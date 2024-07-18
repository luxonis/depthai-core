#include <catch2/catch_all.hpp>

#include <depthai/depthai.hpp>
#include <depthai/utility/SharedMemory.hpp>

#if defined(__unix__) && !defined(__APPLE__)
#include <sys/mman.h>

TEST_CASE("Test Shared Memory FDs") {
    dai::Pipeline pipeline;
    auto xout = pipeline.create<dai::node::XLinkOut>();
    xout->setStreamName("out");

    auto xin = pipeline.create<dai::node::XLinkIn>();
    xin->setStreamName("in");

    xin->out.link(xout->input);

    dai::Device device(pipeline);

    auto inQ = device.getInputQueue("in");
    auto outQ = device.getOutputQueue("out");

    long fd = memfd_create("Test", 0);
    REQUIRE(fd > 0);
    const char *msg = "Hello, world!";
    int rc = write(fd, msg, strlen(msg) + 1);
    REQUIRE(rc > 0);

    auto bufTs = std::chrono::steady_clock::now() + std::chrono::milliseconds(100);
    auto buf = std::make_shared<dai::Buffer>();
    buf->setData(fd);
    buf->setTimestamp(bufTs);

    inQ->send(buf);
    auto out = outQ->get<dai::Buffer>();

    REQUIRE(out != nullptr);

    auto memoryPtr = std::dynamic_pointer_cast<dai::SharedMemory>(out->data);
    REQUIRE(memoryPtr != nullptr);
    REQUIRE(memoryPtr->getFd() > 0);

    const char rcvBuf[256] = {};
    rc = read(memoryPtr->getFd(), rcvBuf, strlen(msg) + 1);
    REQUIRE(rc > 0);
    REQUIRE(strcmp(rcvBuf, msg) == 0);
}
#endif
