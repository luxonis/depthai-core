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
	
const long size = 4096;

class TestSource : public dai::NodeCRTP<dai::node::ThreadedHostNode, TestSource> {
   public:
    Output output = dai::Node::Output{*this, {}};

    void run() override {
	long fd = memfd_create("Test", 0);
        REQUIRE(fd > 0);

	ftruncate(fd, size);
	void *inFile = mmap(NULL, size, PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0);
	REQUIRE(inFile != MAP_FAILED);
	*(uint8_t*)inFile = 0xFF;
	munmap(inFile, size);
    
        auto buffer = std::make_shared<dai::Buffer>();
        buffer->setData(fd);
	buffer->data->setSize(size);

	auto memoryPtr = std::dynamic_pointer_cast<dai::SharedMemory>(buffer->data);
        REQUIRE(memoryPtr != nullptr);
        REQUIRE(memoryPtr->getFd() > 0);

        output.send(buffer);

    }
};

class TestPassthrough : public dai::NodeCRTP<dai::node::ThreadedHostNode, TestPassthrough> {
   public:
    Input input = dai::Node::Input{*this, {}};
    Output output = dai::Node::Output{*this, {}};

    void run() override {
        auto buffer = input.get<dai::Buffer>();

	auto memoryPtr = std::dynamic_pointer_cast<dai::SharedMemory>(buffer->data);
        REQUIRE(memoryPtr != nullptr);
        REQUIRE(memoryPtr->getFd() > 0);

	void *medFile = mmap(NULL, size, PROT_READ | PROT_WRITE, MAP_SHARED, memoryPtr->getFd(), 0);
	REQUIRE(medFile != MAP_FAILED);
	REQUIRE(*(uint8_t*)medFile == 0xFF);
	munmap(medFile, size);

	output.send(buffer);
    }
};

class TestSink : public dai::NodeCRTP<dai::node::ThreadedHostNode, TestSink> {
   public:
    Input input = dai::Node::Input{*this, {}};

    void run() override {
	auto buffer = input.get<dai::Buffer>();
        REQUIRE(buffer != nullptr);

        auto memoryPtr = std::dynamic_pointer_cast<dai::SharedMemory>(buffer->data);
        REQUIRE(memoryPtr != nullptr);
        REQUIRE(memoryPtr->getFd() > 0);

	void *outFile = mmap(NULL, size, PROT_READ | PROT_WRITE, MAP_SHARED, memoryPtr->getFd(), 0);
	REQUIRE(outFile != MAP_FAILED);
	REQUIRE(*(uint8_t*)outFile == 0xFF);
	munmap(outFile, size);

	{
            std::unique_lock<std::mutex> l(mtx);
            done = true;
        }
        cond.notify_one();
    }
};

TEST_CASE("Test Shared Memory FDs") {
    dai::Pipeline pipeline(false);

    auto source = pipeline.create<TestSource>();
    auto passthrough = pipeline.create<TestPassthrough>();
    auto sink = pipeline.create<TestSink>();

    source->output.link(passthrough->input);
    passthrough->output.link(sink->input);
    
    pipeline.start();

    {
        std::unique_lock<std::mutex> l(mtx);
        cond.wait(l, [&done]{ return done; });
    }
}
#endif
