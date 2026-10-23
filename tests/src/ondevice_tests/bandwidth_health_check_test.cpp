#include <algorithm>
#include <catch2/catch_all.hpp>
#include <chrono>
#include <cstdint>
#include <iomanip>
#include <iostream>
#include <limits>
#include <memory>
#include <vector>

#include "depthai/depthai.hpp"

namespace {

// Same load as the bandwidth step of dai::Device::performHealthCheck (src/device/HealthCheck.cpp)
constexpr std::uint32_t FRAME_WIDTH = 1024;
constexpr std::uint32_t FRAME_HEIGHT = 1024;
constexpr std::uint32_t FRAME_BYTES = FRAME_WIDTH * FRAME_HEIGHT;
constexpr std::uint32_t REPORT_EVERY_N_MESSAGES = 128;

constexpr auto TEST_DURATION = std::chrono::seconds(300);
constexpr auto REPORT_TIMEOUT = std::chrono::seconds(10);

float toMbps(float fps) {
    return fps * static_cast<float>(FRAME_BYTES) * 8.0f / 1000000.0f;
}

}  // namespace

// Runs the health check bandwidth measurement for TEST_DURATION and logs each report.
// The test passes when nothing crashes; the bandwidth values are only logged.
TEST_CASE("Bandwidth health check for 300 seconds") {
    auto device = std::make_shared<dai::Device>();
    if(device->getPlatform() != dai::Platform::RVC4) {
        SKIP("The bandwidth health check test is for RVC4 only");
    }
    std::cout << "Device: " << device->getDeviceInfo().toString() << ", USB speed: " << device->getUsbSpeed() << std::endl;

    dai::Pipeline pipeline(device);

    auto benchmarkOut = pipeline.create<dai::node::BenchmarkOut>();
    benchmarkOut->setRunOnHost(false);
    benchmarkOut->setFps(0.0f);

    auto benchmarkIn = pipeline.create<dai::node::BenchmarkIn>();
    benchmarkIn->setRunOnHost(true);
    benchmarkIn->sendReportEveryNMessages(REPORT_EVERY_N_MESSAGES);
    benchmarkIn->logReportsAsWarnings(false);

    benchmarkOut->out.link(benchmarkIn->input);
    auto inputQueue = benchmarkOut->input.createInputQueue();
    auto reportQueue = benchmarkIn->report.createOutputQueue(8, false);

    pipeline.start();

    auto frame = std::make_shared<dai::ImgFrame>();
    frame->setType(dai::ImgFrame::Type::RAW8);
    frame->setSize(FRAME_WIDTH, FRAME_HEIGHT);
    frame->setData(std::vector<std::uint8_t>(FRAME_BYTES, 0xAA));
    inputQueue->send(frame);

    std::cout << std::fixed << std::setprecision(1);

    int reports = 0;
    double messagesReceived = 0.0;
    double timeTotal = 0.0;
    float minMbps = std::numeric_limits<float>::max();
    float maxMbps = 0.0f;

    const auto start = std::chrono::steady_clock::now();
    while(std::chrono::steady_clock::now() - start < TEST_DURATION) {
        bool timedOut = false;
        auto report = reportQueue->get<dai::BenchmarkReport>(REPORT_TIMEOUT, timedOut);
        const std::chrono::duration<double> elapsed = std::chrono::steady_clock::now() - start;
        if(timedOut || report == nullptr) {
            std::cout << "[" << elapsed.count() << " s] No bandwidth report in " << REPORT_TIMEOUT.count() << " s" << std::endl;
            continue;
        }

        const float mbps = toMbps(report->fps);
        reports++;
        messagesReceived += report->numMessagesReceived;
        timeTotal += report->timeTotal;
        minMbps = std::min(minMbps, mbps);
        maxMbps = std::max(maxMbps, mbps);
        std::cout << "[" << elapsed.count() << " s] Bandwidth: " << mbps << " Mbps (" << report->fps << " fps)" << std::endl;
    }

    pipeline.stop();
    pipeline.wait();

    const float avgMbps = timeTotal > 0.0 ? toMbps(static_cast<float>(messagesReceived / timeTotal)) : 0.0f;
    if(reports == 0) {
        minMbps = 0.0f;
    }
    std::cout << "BANDWIDTH_SUMMARY reports=" << reports << " min_mbps=" << minMbps << " avg_mbps=" << avgMbps << " max_mbps=" << maxMbps << std::endl;
}
