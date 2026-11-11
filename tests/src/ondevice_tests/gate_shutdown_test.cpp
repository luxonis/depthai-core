#include <catch2/catch_all.hpp>
#include <chrono>
#include <thread>
#include <vector>

#include "buffer_test_utils.hpp"
#include "depthai/depthai.hpp"
#include "depthai/pipeline/node/internal/XLinkOut.hpp"
#include "depthai/xlink/XLinkStream.hpp"

using namespace buffer_test;

namespace {

void requirePromptClose(dai::Device& device) {
    const auto start = std::chrono::steady_clock::now();
    device.close();
    const auto closeTime = std::chrono::steady_clock::now() - start;
    REQUIRE(closeTime < std::chrono::seconds(10));
    REQUIRE(device.isClosed());
}

}  // namespace

TEST_CASE("Device closes promptly with an idle Gate input") {
    dai::Pipeline pipeline;
    auto device = pipeline.getDefaultDevice();
    auto gate = pipeline.create<dai::node::Gate>();
    gate->setRunOnHost(false);
    gate->initialConfig->open = true;
    auto input = gate->input.createInputQueue();
    auto output = gate->output.createOutputQueue();
    pipeline.start();

    input->send(makeBuffer(81));
    requireBuffer(take<dai::Buffer>(output), 81);
    requireNoMessage(output);
    requirePromptClose(*device);
}

TEST_CASE("Unread device output leaves RPC and close responsive") {
    dai::Pipeline pipeline;
    auto device = pipeline.getDefaultDevice();
    auto source = pipeline.create<dai::node::BenchmarkOut>();
    source->setRunOnHost(false);
    source->setFps(100);
    auto input = source->input.createInputQueue();
    // A raw stream with no host queue: nothing drains it after the first read
    auto out = pipeline.create<dai::node::internal::XLinkOut>();
    out->setStreamName("unread_output");
    source->out.link(out->input);
    pipeline.start();

    dai::XLinkStream unread(device->getConnection(), "unread_output", 1);
    auto seed = makeBuffer(71);
    seed->setData(std::vector<uint8_t>(1024 * 1024, 0xa5));
    input->send(seed);
    const auto first = unread.read(std::chrono::seconds(2));
    REQUIRE(first.size() > 1024 * 1024);
    REQUIRE(first.front() == 0xa5);

    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    REQUIRE(device->getDeviceId() == device->getDeviceInfo().getDeviceId());
    requirePromptClose(*device);
}
