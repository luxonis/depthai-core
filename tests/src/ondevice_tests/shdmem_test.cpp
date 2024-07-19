#include <catch2/catch_all.hpp>

#include <depthai/depthai.hpp>
#include <depthai/utility/SharedMemory.hpp>

#if defined(__unix__) && !defined(__APPLE__)
#include <sys/mman.h>

#include <chrono>
#include <iostream>
#include <thread>

#include "depthai/depthai.hpp"

std::mutex mtx;
std::condition_variable cond;
bool done = false;

class TestSink : public dai::NodeCRTP<dai::node::ThreadedHostNode, TestSink> {
   public:
    Input input = dai::Node::Input{*this, {}};

    void run() override {
	auto buffer = input.get<dai::Buffer>();
        REQUIRE(buffer != nullptr);

        auto memoryPtr = std::dynamic_pointer_cast<dai::SharedMemory>(buffer->data);
        REQUIRE(memoryPtr != nullptr);
        REQUIRE(memoryPtr->getFd() > 0);

	{
            std::unique_lock<std::mutex> l(mtx);
            done = true;
        }
        cond.notify_one();
    }
};

class TestSource : public dai::NodeCRTP<dai::node::ThreadedHostNode, TestSource> {
   public:
    Output output = dai::Node::Output{*this, {}};

    void run() override {
	long fd = memfd_create("Test", 0);
        REQUIRE(fd > 0);

        auto buffer = std::make_shared<dai::Buffer>();
        buffer->setData(fd);
        output.send(buffer);
    }
};

TEST_CASE("Test Shared Memory FDs") {
    dai::Pipeline pipeline(false);

    auto source = pipeline.create<TestSource>();
    auto sink = pipeline.create<TestSink>();

    source->output.link(sink->input);
    
    pipeline.start();

    {
        std::unique_lock<std::mutex> l(mtx);
        cond.wait(l, [&done]{ return done; });

    }
}
#endif
