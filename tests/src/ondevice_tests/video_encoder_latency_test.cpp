#include <algorithm>
#include <catch2/catch_all.hpp>
#include <chrono>
#include <numeric>
#include <optional>
#include <thread>
#include <vector>

#include "depthai/depthai.hpp"

namespace {

std::optional<dai::CameraBoardSocket> findHighResolutionColorCamera(const std::shared_ptr<dai::Device>& device) {
    for(const auto& features : device->getConnectedCameraFeatures()) {
        const bool isColor =
            std::find(features.supportedTypes.begin(), features.supportedTypes.end(), dai::CameraSensorType::COLOR) != features.supportedTypes.end();
        if(isColor && features.width >= 4000 && features.height >= 3000) {
            return features.socket;
        }
    }
    return std::nullopt;
}

void testVideoEncoderLatency(dai::VideoEncoderProperties::Profile profile, double maxAverageLatency, double maxP95Latency) {
    dai::Pipeline pipeline;
    const auto device = pipeline.getDefaultDevice();
    if(device->getPlatform() != dai::Platform::RVC4) {
        SKIP("The latency thresholds cover the RVC4 VideoEncoder implementation");
    }

    const auto cameraSocket = findHighResolutionColorCamera(device);
    if(!cameraSocket.has_value()) {
        SKIP("A color camera supporting 4000x3000 is required");
    }

    constexpr float cameraFps = 30.0f;
    auto camera = pipeline.create<dai::node::Camera>()->build(cameraSocket.value());
    auto cameraOutput = camera->requestOutput({4000, 3000}, dai::ImgFrame::Type::NV12, dai::ImgResizeMode::CROP, cameraFps);

    auto encoder = pipeline.create<dai::node::VideoEncoder>();
    encoder->setDefaultProfilePreset(cameraFps, profile);
    encoder->input.setBlocking(false);
    encoder->input.setMaxSize(1);
    if(profile != dai::VideoEncoderProperties::Profile::MJPEG) {
        encoder->setFrameRate(120.0f);
        encoder->setKeyframeFrequency(static_cast<int>(cameraFps));
        encoder->setBitrate(20000000);
        encoder->setRateControlMode(dai::VideoEncoderProperties::RateControlMode::CBR);
    }

    auto benchmark = pipeline.create<dai::node::BenchmarkIn>();
    benchmark->setRunOnHost(false);
    benchmark->sendReportEveryNMessages(30);
    benchmark->measureIndividualLatencies(true);
    benchmark->logReportsAsWarnings(false);

    cameraOutput->link(encoder->input);
    encoder->out.link(benchmark->input);
    auto reportQueue = benchmark->report.createOutputQueue(4, false);

    pipeline.start();
    std::vector<double> averageLatencies;
    std::vector<double> individualLatencies;
    std::vector<float> measuredFps;
    int reportCount = 0;
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(15);
    while(reportCount < 4 && std::chrono::steady_clock::now() < deadline) {
        auto report = reportQueue->tryGet<dai::BenchmarkReport>();
        if(report == nullptr) {
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
            continue;
        }
        if(reportCount > 0) {
            averageLatencies.push_back(report->averageLatency);
            individualLatencies.insert(individualLatencies.end(), report->latencies.begin(), report->latencies.end());
            measuredFps.push_back(report->fps);
        }
        ++reportCount;
    }
    pipeline.stop();
    pipeline.wait();

    REQUIRE(reportCount == 4);
    REQUIRE(averageLatencies.size() == 3);
    REQUIRE(individualLatencies.size() >= 90);
    for(const auto fps : measuredFps) {
        REQUIRE(fps > 28.0f);
        REQUIRE(fps < 31.0f);
    }

    const double averageLatency = std::accumulate(averageLatencies.begin(), averageLatencies.end(), 0.0) / averageLatencies.size();
    std::sort(individualLatencies.begin(), individualLatencies.end());
    const double p95Latency = individualLatencies[static_cast<std::size_t>(individualLatencies.size() * 0.95)];
    CAPTURE(averageLatency, p95Latency);
    REQUIRE(averageLatency < maxAverageLatency);
    REQUIRE(p95Latency < maxP95Latency);
}

}  // namespace

TEST_CASE("RVC4 VideoEncoder H264 4000x3000 latency regression") {
    testVideoEncoderLatency(dai::VideoEncoderProperties::Profile::H264_MAIN, 0.085, 0.100);
}

TEST_CASE("RVC4 VideoEncoder H265 4000x3000 latency regression") {
    testVideoEncoderLatency(dai::VideoEncoderProperties::Profile::H265_MAIN, 0.085, 0.100);
}

TEST_CASE("RVC4 VideoEncoder MJPEG 4000x3000 latency regression") {
    testVideoEncoderLatency(dai::VideoEncoderProperties::Profile::MJPEG, 0.090, 0.110);
}
