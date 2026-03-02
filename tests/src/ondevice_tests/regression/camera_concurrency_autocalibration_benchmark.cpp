#include <catch2/catch_all.hpp>

#include <cstdlib>
#include <iostream>
#include <memory>
#include <numeric>
#include <vector>

#include "depthai/depthai.hpp"
#include "depthai/pipeline/datatype/BenchmarkReport.hpp"
#include "depthai/pipeline/node/BenchmarkIn.hpp"

TEST_CASE("camera_concurrency_autocalibration_benchmark") {
    constexpr float FPS = 30.0f;
    constexpr int SECONDS_TO_RUN = 50;
    constexpr float MAX_FPS_OFFSET = 0.5f;
    constexpr int MAX_SECONDS_OFFSET = 5;
    constexpr int NUM_MESSAGES_RELAXED = 500;
    constexpr int TIMEOUT_SECONDS_RELAXED = 40;

    const char* autoCalibrationEnv = std::getenv("DEPTHAI_AUTOCALIBRATION");
    const std::string autoCalibrationMode = autoCalibrationEnv ? autoCalibrationEnv : "";
    const bool strictMode = (std::getenv("AUTOCAL_BENCH_STRICT") != nullptr) && (std::string(std::getenv("AUTOCAL_BENCH_STRICT")) == "1");

    std::cout << "[AUTOCAL_BENCH] DEPTHAI_AUTOCALIBRATION=" << (autoCalibrationMode.empty() ? "<unset>" : autoCalibrationMode) << std::endl;
    std::cout << "[AUTOCAL_BENCH] strict_mode=" << (strictMode ? "1" : "0") << std::endl;

    dai::Pipeline pipeline;
    auto cam = pipeline.create<dai::node::Camera>()->build(dai::CameraBoardSocket::CAM_A);
    auto monoLeft = pipeline.create<dai::node::Camera>()->build(dai::CameraBoardSocket::CAM_B);
    auto monoRight = pipeline.create<dai::node::Camera>()->build(dai::CameraBoardSocket::CAM_C);

    std::vector<dai::Node::Output*> cameraOutputs = {
        cam->requestOutput(std::make_pair(640, 352), dai::ImgFrame::Type::BGR888i, dai::ImgResizeMode::CROP, FPS),
        cam->requestOutput(std::make_pair(1280, 704), dai::ImgFrame::Type::NV12, dai::ImgResizeMode::CROP, FPS),
        monoLeft->requestOutput(std::make_pair(1280, 720), dai::ImgFrame::Type::NV12, dai::ImgResizeMode::CROP, FPS),
        monoRight->requestOutput(std::make_pair(1280, 720), dai::ImgFrame::Type::NV12, dai::ImgResizeMode::CROP, FPS),
    };

    std::vector<std::shared_ptr<dai::MessageQueue>> queues;
    std::vector<std::shared_ptr<dai::node::BenchmarkIn>> benchmarkNodes;
    const int numMessagesToGet = strictMode ? static_cast<int>(FPS * static_cast<float>(SECONDS_TO_RUN)) : NUM_MESSAGES_RELAXED;
    const int timeoutSeconds = strictMode ? (SECONDS_TO_RUN + MAX_SECONDS_OFFSET) : TIMEOUT_SECONDS_RELAXED;

    for(auto* output : cameraOutputs) {
        auto node = pipeline.create<dai::node::BenchmarkIn>();
        output->link(node->input);
        node->sendReportEveryNMessages(numMessagesToGet);
        queues.push_back(node->report.createOutputQueue());
        benchmarkNodes.push_back(node);
    }

    pipeline.start();

    std::vector<float> streamFps;
    streamFps.reserve(queues.size());
    int idx = 0;
    for(auto& queue : queues) {
        bool timedOut = false;
        std::cout << "Getting output: " << idx << "\n" << std::flush;
        auto report = queue->get<dai::BenchmarkReport>(std::chrono::seconds(timeoutSeconds), timedOut);
        std::cout << "Getting output: " << idx << " timed out: " << (timedOut ? "true" : "false") << "\n" << std::flush;

        REQUIRE(!timedOut);
        REQUIRE(report);
        REQUIRE(report->numMessagesReceived == numMessagesToGet);
        if(strictMode) {
            REQUIRE(report->fps >= FPS - MAX_FPS_OFFSET);
            REQUIRE(report->fps <= FPS + MAX_FPS_OFFSET);
        }

        streamFps.push_back(report->fps);
        std::cout << "[AUTOCAL_BENCH] mode=" << (autoCalibrationMode.empty() ? "<unset>" : autoCalibrationMode) << " stream=" << idx
                  << " fps=" << report->fps << " messages=" << report->numMessagesReceived << std::endl;
        ++idx;
    }

    const float avgFps = std::accumulate(streamFps.begin(), streamFps.end(), 0.0f) / static_cast<float>(streamFps.size());
    auto [minIt, maxIt] = std::minmax_element(streamFps.begin(), streamFps.end());
    std::cout << "[AUTOCAL_BENCH] mode=" << (autoCalibrationMode.empty() ? "<unset>" : autoCalibrationMode) << " avg_fps=" << avgFps
              << " min_fps=" << *minIt << " max_fps=" << *maxIt << std::endl;

    REQUIRE(avgFps > 1.0f);
}
