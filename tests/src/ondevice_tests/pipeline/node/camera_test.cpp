#include "depthai/pipeline/node/Camera.hpp"

#include <catch2/catch_all.hpp>
#include <catch2/catch_test_macros.hpp>
#include <catch2/generators/catch_generators.hpp>
#include <chrono>
#include <iostream>
#include <thread>

#include "depthai/capabilities/ImgFrameCapability.hpp"
#include "depthai/common/CameraBoardSocket.hpp"
#include "depthai/common/CameraFeatures.hpp"
#include "depthai/depthai.hpp"
#include "depthai/pipeline/MessageQueue.hpp"
#include "depthai/pipeline/datatype/BenchmarkReport.hpp"
#include "depthai/xlink/XLinkConnection.hpp"

TEST_CASE("Test raw camera output") {
    // Create pipeline
    dai::Pipeline p;
    auto fpsToRun = GENERATE(10.5, 20.0, 30.0);
    auto camera = p.create<dai::node::Camera>()->build(dai::CameraBoardSocket::AUTO, std::nullopt, fpsToRun);
    auto benchmarkIn = p.create<dai::node::BenchmarkIn>();
    SECTION("RAW") {
        camera->raw.link(benchmarkIn->input);
    }
    SECTION("NORMAL") {
        auto* output = camera->requestFullResolutionOutput();
        REQUIRE(output != nullptr);
        output->link(benchmarkIn->input);
    }
    benchmarkIn->sendReportEveryNMessages(static_cast<uint32_t>(std::round(fpsToRun) * 2));
    auto benchmarkQueue = benchmarkIn->report.createOutputQueue();

    p.start();

    for(int i = 0; i < 3; i++) {
        auto benchmarkReport = benchmarkQueue->get<dai::BenchmarkReport>();
        // Allow +-10% difference
        REQUIRE(benchmarkReport->fps == Catch::Approx(fpsToRun).margin(fpsToRun * 0.1));
    }
}

TEST_CASE("Test camera with multiple outputs with different FPS") {
    // Create pipeline
    dai::Pipeline p;
    auto camera = p.create<dai::node::Camera>()->build();
    std::map<float, std::shared_ptr<dai::MessageQueue>> fpsToReportQueue;
    bool haveRaw = false;
    std::shared_ptr<dai::MessageQueue> rawQueue;
    // for(auto fps : {10.5f, 30.0f}) { // Only one additional output in case raw is enabled is supported on RVC4
    for(auto fps : {30.0f}) {
        auto benchmarkIn = p.create<dai::node::BenchmarkIn>();
        benchmarkIn->sendReportEveryNMessages(static_cast<uint32_t>(std::round(fps) * 2));
        auto* output = camera->requestOutput(std::make_pair(640, 400), std::nullopt, dai::ImgResizeMode::CROP, fps);
        REQUIRE(output != nullptr);
        output->link(benchmarkIn->input);
        fpsToReportQueue[fps] = benchmarkIn->report.createOutputQueue();
    }

    // TODO(Morato) - add back when fixed on the newest OSes
    // SECTION("With RAW") {
    //     auto benchmarkInRaw = p.create<dai::node::BenchmarkIn>();
    //     camera->raw.link(benchmarkInRaw->input);
    //     rawQueue = benchmarkInRaw->report.createOutputQueue();
    //     haveRaw = true;
    // }

    SECTION("No RAW") {
        haveRaw = false;
    }

    p.start();
    for(int i = 0; i < 3; i++) {
        for(auto& [fps, queue] : fpsToReportQueue) {
            auto benchmarkReport = queue->get<dai::BenchmarkReport>();
            // Allow +-10% difference
            REQUIRE(benchmarkReport->fps == Catch::Approx(fps).margin(fps * 0.1));
        }
        if(haveRaw) {
            auto benchmarkReport = rawQueue->get<dai::BenchmarkReport>();
            // Allow +-10% difference
            REQUIRE(benchmarkReport->fps == Catch::Approx(30.0).margin(30.0 * 0.1));
        }
    }
}

#ifndef _WIN32  // TODO(Jakob) - fix this test on Windows and bring it back on RVC4 when multi FPS is supported
// TEST_CASE("Test setting the center camera to a different FPS compared to left and right") {
//     // Create pipeline
//     dai::Pipeline p;
//     std::map<dai::CameraBoardSocket, float> socketToFps = {
//         {dai::CameraBoardSocket::CAM_A, 10.0},
//         {dai::CameraBoardSocket::CAM_B, 20.0},
//         {dai::CameraBoardSocket::CAM_C, 20.0},
//     };
//     std::map<dai::CameraBoardSocket, std::shared_ptr<dai::MessageQueue>> messageQueues;
//     for(auto& [socket, fps] : socketToFps) {
//         auto camera = p.create<dai::node::Camera>()->build(socket, std::nullopt, fps);
//         auto benchmarkIn = p.create<dai::node::BenchmarkIn>();
//         benchmarkIn->sendReportEveryNMessages(static_cast<uint32_t>(std::round(fps) * 2));
//         auto* output = camera->requestFullResolutionOutput();
//         REQUIRE(output != nullptr);
//         output->link(benchmarkIn->input);
//         messageQueues[socket] = benchmarkIn->report.createOutputQueue();
//     }

//     p.start();
//     for(int i = 0; i < 3; i++) {
//         for(auto& [socket, queue] : messageQueues) {
//             auto benchmarkReport = queue->get<dai::BenchmarkReport>();
//             // Allow +-10% difference
//             REQUIRE(benchmarkReport->fps == Catch::Approx(socketToFps[socket]).margin(socketToFps[socket] * 0.1));
//         }
//     }
// }

TEST_CASE("Multiple outputs") {
    dai::Pipeline p;
    auto camera = p.create<dai::node::Camera>()->build();
    auto* camOut1 = camera->requestOutput({500, 300}, dai::ImgFrame::Type::NV12);
    auto* camOut2 = camera->requestOutput({700, 500}, dai::ImgFrame::Type::RGB888i);
    auto* camOut3 = camera->requestOutput({900, 700}, dai::ImgFrame::Type::GRAY8);

    auto camOut1Queue = camOut1->createOutputQueue();
    auto camOut2Queue = camOut2->createOutputQueue();
    auto camOut3Queue = camOut3->createOutputQueue();

    p.start();
    for(int i = 0; i < 5; i++) {
        auto frame1 = camOut1Queue->get<dai::ImgFrame>();
        REQUIRE(frame1->getWidth() == 500);
        REQUIRE(frame1->getHeight() == 300);
        REQUIRE(frame1->getType() == dai::ImgFrame::Type::NV12);

        auto frame2 = camOut2Queue->get<dai::ImgFrame>();
        REQUIRE(frame2->getWidth() == 700);
        REQUIRE(frame2->getHeight() == 500);
        REQUIRE(frame2->getType() == dai::ImgFrame::Type::RGB888i);

        auto frame3 = camOut3Queue->get<dai::ImgFrame>();
        REQUIRE(frame3->getWidth() == 900);
        REQUIRE(frame3->getHeight() == 700);
        REQUIRE(frame3->getType() == dai::ImgFrame::Type::GRAY8);
    }
}
#endif

TEST_CASE("Camera start/stop stream") {
    constexpr float K_FPS = 30.0f;
    constexpr uint32_t K_REPORT_EVERY_N = 10;
    const auto kReportTimeout = std::chrono::seconds(4);

    dai::Pipeline p;
    auto camera = p.create<dai::node::Camera>()->build();
    camera->initialControl.setStopStreaming();
    auto* output = camera->requestFullResolutionOutput(dai::ImgFrame::Type::NV12, K_FPS);
    REQUIRE(output != nullptr);

    auto benchmarkIn = p.create<dai::node::BenchmarkIn>();
    output->link(benchmarkIn->input);
    benchmarkIn->sendReportEveryNMessages(K_REPORT_EVERY_N);
    auto reportQueue = benchmarkIn->report.createOutputQueue();
    auto controlQueue = camera->inputControl.createInputQueue();

    auto waitForReport = [&](std::chrono::milliseconds timeout) {
        bool timedOut = false;
        auto report = reportQueue->get<dai::BenchmarkReport>(timeout, timedOut);
        return timedOut ? nullptr : report;
    };

    p.start();

    auto startCtrl = std::make_shared<dai::CameraControl>();
    startCtrl->setStartStreaming();
    controlQueue->send(startCtrl);

    auto initialReport = waitForReport(kReportTimeout);
    REQUIRE(initialReport != nullptr);

    while(reportQueue->tryGet<dai::BenchmarkReport>() != nullptr) {
    }

    auto stopCtrl = std::make_shared<dai::CameraControl>();
    stopCtrl->setStopStreaming();
    controlQueue->send(stopCtrl);

    auto next = waitForReport(kReportTimeout);
    REQUIRE(next == nullptr);

    auto restartCtrl = std::make_shared<dai::CameraControl>();
    restartCtrl->setStartStreaming();
    controlQueue->send(restartCtrl);

    auto restartedReport = waitForReport(kReportTimeout);
    REQUIRE(restartedReport != nullptr);
}

TEST_CASE("Test how default FPS is generated for a specific output") {
    constexpr float FPS_TO_SET = 20.0;
    // Create pipeline
    dai::Pipeline p;
    auto camera = p.create<dai::node::Camera>()->build();
    auto benchmarkIn = p.create<dai::node::BenchmarkIn>();
    auto* output1 = camera->requestOutput(std::make_pair(640, 400), std::nullopt);
    REQUIRE(output1 != nullptr);
    auto* output2 = camera->requestOutput(std::make_pair(640, 400), std::nullopt, dai::ImgResizeMode::CROP, FPS_TO_SET);
    REQUIRE(output2 != nullptr);
    output2->createOutputQueue();  // "Sink it"
    output1->link(benchmarkIn->input);
    benchmarkIn->sendReportEveryNMessages(FPS_TO_SET * 2);
    auto benchmarkQueue = benchmarkIn->report.createOutputQueue();

    p.start();
    for(int i = 0; i < 3; i++) {
        auto benchmarkReport = benchmarkQueue->get<dai::BenchmarkReport>();
        // Allow +-10% difference
        REQUIRE(benchmarkReport->fps == Catch::Approx(FPS_TO_SET).margin(FPS_TO_SET * 0.1));
    }
}

TEST_CASE("Camera run without calibration") {
    dai::Pipeline p;
    p.setCalibrationData(dai::CalibrationHandler());  // empty calibration data
    std::vector<std::shared_ptr<dai::MessageQueue>> outputQueues;
    for(auto socket : p.getDefaultDevice()->getConnectedCameras()) {
        auto camera = p.create<dai::node::Camera>()->build(socket);
        auto* output = camera->requestFullResolutionOutput();
        REQUIRE(output != nullptr);
        outputQueues.emplace_back(output->createOutputQueue());
    }
    p.start();
    for(auto& queue : outputQueues) {
        auto frame = queue->get<dai::ImgFrame>();
        REQUIRE(frame->getWidth() > 0);
        REQUIRE(frame->getHeight() > 0);
    }
}
