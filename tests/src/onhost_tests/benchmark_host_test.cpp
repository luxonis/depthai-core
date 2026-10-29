#include <catch2/catch_test_macros.hpp>
#include <chrono>

#include "depthai/pipeline/Pipeline.hpp"
#include "depthai/pipeline/node/BenchmarkOut.hpp"

TEST_CASE("Stopping a slow host BenchmarkOut interrupts its pacing wait", "[benchmark][stop]") {
    using namespace std::chrono_literals;
    dai::Pipeline pipeline(false);
    auto benchmark = pipeline.create<dai::node::BenchmarkOut>();
    benchmark->setRunOnHost(true);
    benchmark->setFps(0.01f);
    auto input = benchmark->input.createInputQueue();
    auto output = benchmark->out.createOutputQueue();
    auto message = std::make_shared<dai::Buffer>();
    message->setData({0x12, 0x34});
    pipeline.start();
    input->send(message);
    bool timedOut = false;
    auto received = output->get<dai::Buffer>(1s, timedOut);
    REQUIRE_FALSE(timedOut);
    REQUIRE(received);
    REQUIRE(std::vector<uint8_t>(received->getData().begin(), received->getData().end()) == std::vector<uint8_t>{0x12, 0x34});
    const auto stoppedAt = std::chrono::steady_clock::now();
    pipeline.stop();
    pipeline.wait();
    REQUIRE(std::chrono::steady_clock::now() - stoppedAt < 1s);
}

TEST_CASE("Host BenchmarkOut sends repeated payloads at its configured pace", "[benchmark][pacing]") {
    using namespace std::chrono_literals;
    dai::Pipeline pipeline(false);
    auto benchmark = pipeline.create<dai::node::BenchmarkOut>();
    benchmark->setRunOnHost(true);
    benchmark->setFps(5.0f);
    benchmark->setNumMessagesToSend(2);
    auto input = benchmark->input.createInputQueue();
    auto output = benchmark->out.createOutputQueue();
    auto message = std::make_shared<dai::Buffer>();
    message->setData({0x56, 0x78});
    pipeline.start();
    const auto startedAt = std::chrono::steady_clock::now();
    input->send(message);
    for(int i = 0; i < 2; ++i) {
        bool timedOut = false;
        auto received = output->get<dai::Buffer>(1s, timedOut);
        REQUIRE_FALSE(timedOut);
        REQUIRE(received);
        REQUIRE(std::vector<uint8_t>(received->getData().begin(), received->getData().end()) == std::vector<uint8_t>{0x56, 0x78});
    }
    REQUIRE(std::chrono::steady_clock::now() - startedAt >= 200ms);
    pipeline.stop();
    pipeline.wait();
}
