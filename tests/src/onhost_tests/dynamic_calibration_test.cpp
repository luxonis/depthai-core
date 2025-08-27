#include <atomic>
#include <catch2/catch_all.hpp>
#include <chrono>
#include <depthai/depthai.hpp>
#include <memory>
#include <thread>

// Nodes
#include "depthai/pipeline/node/DynamicCalibrationNode.hpp"  // provides dai::node::DynamicCalibration

using namespace std::chrono_literals;

namespace {

dai::Pipeline makePipeline(const std::shared_ptr<dai::Device>& device,
                           std::shared_ptr<dai::node::DynamicCalibration>& dynCalibOut,
                           std::shared_ptr<dai::node::StereoDepth>& stereoOut) {
    // Construct pipeline bound to the device
    dai::Pipeline p(device);

    // Cameras via .build(socket)
    auto camLeft = p.create<dai::node::Camera>()->build(dai::CameraBoardSocket::CAM_B);
    auto camRight = p.create<dai::node::Camera>()->build(dai::CameraBoardSocket::CAM_C);

    // Full-res NV12 outputs; NOTE: these return pointers
    auto* leftOut = camLeft->requestFullResolutionOutput(dai::ImgFrame::Type::NV12);
    auto* rightOut = camRight->requestFullResolutionOutput(dai::ImgFrame::Type::NV12);

    // DynamicCalibration
    auto dynCalib = p.create<dai::node::DynamicCalibration>();
    leftOut->link(dynCalib->left);
    rightOut->link(dynCalib->right);

    // StereoDepth (sanity streams)
    auto stereo = p.create<dai::node::StereoDepth>();
    leftOut->link(stereo->left);
    rightOut->link(stereo->right);

    // Return nodes to caller
    dynCalibOut = dynCalib;
    stereoOut = stereo;
    return p;
}

}  // namespace

TEST_CASE("DynamicCalibration reaches a result and applies only when ready") {
    // Real device (on-host test)
    auto device = std::make_shared<dai::Device>();
    REQUIRE(device != nullptr);

    // Optional: capture WARN/ERROR logs
    std::atomic<bool> sawWarnOrError{false};
    device->setLogLevel(dai::LogLevel::WARN);
    device->addLogCallback([&](const dai::LogMessage& m) {
        if(m.level >= dai::LogLevel::WARN) sawWarnOrError = true;
    });

    // Pipeline + nodes
    std::shared_ptr<dai::node::DynamicCalibration> dynCalib;
    std::shared_ptr<dai::node::StereoDepth> stereo;
    auto pipeline = makePipeline(device, dynCalib, stereo);
    REQUIRE(dynCalib);
    REQUIRE(stereo);

    // Queues (shared_ptrs)
    auto calibration_output = dynCalib->calibrationOutput.createOutputQueue();
    auto coverage_output = dynCalib->coverageOutput.createOutputQueue();
    auto config_input = dynCalib->inputConfig.createInputQueue();
    auto command_input = dynCalib->inputControl.createInputQueue();

    auto left_xout = stereo->syncedLeft.createOutputQueue();
    auto right_xout = stereo->syncedRight.createOutputQueue();
    auto disp_xout = stereo->disparity.createOutputQueue();

    // Apply current calibration (like your example)
    auto calib_data = device->readCalibration();
    device->setCalibration(calib_data);

    // Start the pipeline
    pipeline.start();

    // Give AE a moment
    std::this_thread::sleep_for(1s);

    // Sanity: we should be able to fetch some frames (blocking get<...>())
    {
        auto l = left_xout->get<dai::ImgFrame>();
        auto r = right_xout->get<dai::ImgFrame>();
        auto d = disp_xout->get<dai::ImgFrame>();
        REQUIRE(l != nullptr);
        REQUIRE(r != nullptr);
        REQUIRE(d != nullptr);
    }

    // Kick off recalibration (matching your sample)
    command_input->send(std::make_shared<dai::StartCalibrationCommand>(dai::DynamicCalibrationConfig::PerformanceMode::OPTIMIZE_PERFORMANCE));

    bool completed = false;
    float lastCoverage = 0.0f;
    const int kMaxIterations = 200;  // safeguard for CI/lab scenes

    for(int i = 0; i < kMaxIterations && pipeline.isRunning(); ++i) {
        // Block for coverage update and result
        auto coverage = coverage_output->get<dai::CoverageData>();
        REQUIRE(coverage != nullptr);
        INFO("Iteration " << i << " meanCoverage=" << coverage->meanCoverage);
        REQUIRE(coverage->meanCoverage >= lastCoverage - 1e-4f);  // non-decreasing (tolerate float jitter)
        lastCoverage = coverage->meanCoverage;

        auto result = calibration_output->get<dai::DynamicCalibrationResult>();
        REQUIRE(result != nullptr);

        if(result->calibrationData.has_value()) {
            // We expect to see a payload only when the process is complete
            completed = true;

            // Optional: immediately apply it (like your example)
            command_input->send(std::make_shared<dai::ApplyCalibrationCommand>(result->calibrationData->newCalibration));
            break;
        } else {
            // While running, info should be non-empty (typically progress/status text)
            REQUIRE(!result->info.empty());
        }
    }

    REQUIRE(completed);
    REQUIRE_FALSE(sawWarnOrError);

    pipeline.stop();
    pipeline.wait();
}

TEST_CASE("DynamicCalibration: empty-data requests yield no calibration/quality payloads") {
    auto device = std::make_shared<dai::Device>();
    REQUIRE(device != nullptr);

    // Optional: watch for WARN/ERROR logs
    std::atomic<bool> sawWarnOrError{false};
    device->setLogLevel(dai::LogLevel::WARN);
    device->addLogCallback([&](const dai::LogMessage& m) {
        if(m.level >= dai::LogLevel::WARN) sawWarnOrError = true;
    });

    // Build pipeline
    std::shared_ptr<dai::node::DynamicCalibration> dynCalib;
    std::shared_ptr<dai::node::StereoDepth> stereo;
    auto pipeline = makePipeline(device, dynCalib, stereo);
    REQUIRE(dynCalib);
    REQUIRE(stereo);

    // Queues
    auto calibration_output = dynCalib->calibrationOutput.createOutputQueue();
    auto quality_output = dynCalib->qualityOutput.createOutputQueue();
    auto command_input = dynCalib->inputControl.createInputQueue();

    auto left_xout = stereo->syncedLeft.createOutputQueue();
    auto right_xout = stereo->syncedRight.createOutputQueue();
    auto disp_xout = stereo->disparity.createOutputQueue();

    // Apply current calibration
    device->setCalibration(device->readCalibration());

    // Start; brief AE settle
    pipeline.start();
    std::this_thread::sleep_for(1s);

    // (Optional) verify stream is alive
    REQUIRE(left_xout->get<dai::ImgFrame>() != nullptr);
    REQUIRE(right_xout->get<dai::ImgFrame>() != nullptr);
    REQUIRE(disp_xout->get<dai::ImgFrame>() != nullptr);

    // 1) Calibrate(performanceMode=OPTIMIZE_PERFORMANCE) -> expect no calibrationData
    {
        auto cmd = std::make_shared<dai::CalibrateCommand>();
        cmd->performanceMode = dai::DynamicCalibrationConfig::PerformanceMode::OPTIMIZE_PERFORMANCE;
        command_input->send(cmd);

        auto result = calibration_output->get<dai::DynamicCalibrationResult>();
        REQUIRE(result != nullptr);
        INFO("Calibrate #1 info: " << result->info);
        REQUIRE_FALSE(result->calibrationData.has_value());
    }

    // 2) Calibrate(force=true) -> still expect no calibrationData with insufficient data
    {
        auto cmd = std::make_shared<dai::CalibrateCommand>();
        cmd->force = true;
        command_input->send(cmd);

        auto result = calibration_output->get<dai::DynamicCalibrationResult>();
        REQUIRE(result != nullptr);
        INFO("Calibrate #2 (force) info: " << result->info);
        REQUIRE_FALSE(result->calibrationData.has_value());
    }

    // 3) CalibrationQuality(force=true) -> expect data == null
    {
        auto qcmd = std::make_shared<dai::CalibrationQualityCommand>();
        qcmd->force = true;
        command_input->send(qcmd);

        auto qres = quality_output->get<dai::CalibrationQuality>();
        REQUIRE(qres != nullptr);
        INFO("Quality #1 (force) info: " << qres->info);
        REQUIRE_FALSE(qres->data.has_value());
    }

    // 4) CalibrationQuality(force=false) -> expect data == null
    {
        auto qcmd = std::make_shared<dai::CalibrationQualityCommand>();
        qcmd->force = false;
        command_input->send(qcmd);

        auto qres = quality_output->get<dai::CalibrationQuality>();
        REQUIRE(qres != nullptr);
        INFO("Quality #2 (no force) info: " << qres->info);
        REQUIRE_FALSE(qres->data.has_value());
    }

    REQUIRE_FALSE(sawWarnOrError);

    pipeline.stop();
    pipeline.wait();
}

TEST_CASE("DynamicCalibration: StopCalibration halts further results") {
    auto device = std::make_shared<dai::Device>();
    REQUIRE(device != nullptr);

    // Optional: trap WARN/ERROR logs
    std::atomic<bool> sawWarnOrError{false};
    device->setLogLevel(dai::LogLevel::WARN);
    device->addLogCallback([&](const dai::LogMessage& m) {
        if(m.level >= dai::LogLevel::WARN) sawWarnOrError = true;
    });

    // Build pipeline
    std::shared_ptr<dai::node::DynamicCalibration> dynCalib;
    std::shared_ptr<dai::node::StereoDepth> stereo;
    auto pipeline = makePipeline(device, dynCalib, stereo);
    REQUIRE(dynCalib);
    REQUIRE(stereo);

    // Queues
    auto calibration_output = dynCalib->calibrationOutput.createOutputQueue();
    auto coverage_output = dynCalib->coverageOutput.createOutputQueue();
    auto command_input = dynCalib->inputControl.createInputQueue();

    // (Optional) sanity streams
    auto left_xout = stereo->syncedLeft.createOutputQueue();
    auto right_xout = stereo->syncedRight.createOutputQueue();
    auto disp_xout = stereo->disparity.createOutputQueue();

    // Apply current calibration
    device->setCalibration(device->readCalibration());

    // Start and let AE settle
    pipeline.start();
    std::this_thread::sleep_for(1s);

    // (Optional) ensure streams are alive
    REQUIRE(left_xout->get<dai::ImgFrame>() != nullptr);
    REQUIRE(right_xout->get<dai::ImgFrame>() != nullptr);
    REQUIRE(disp_xout->get<dai::ImgFrame>() != nullptr);

    // Start recalibration (performance-optimized)
    command_input->send(std::make_shared<dai::StartCalibrationCommand>(dai::DynamicCalibrationConfig::PerformanceMode::OPTIMIZE_PERFORMANCE));

    // Pull the first result (whatever it is—likely just info with no payload yet)
    auto first = calibration_output->get<dai::DynamicCalibrationResult>();
    REQUIRE(first != nullptr);

    // Now stop the recalibration
    command_input->send(std::make_shared<dai::StopCalibrationCommand>());

    // Immediately try a non-blocking poll (may or may not have an in-flight message)
    (void)calibration_output->tryGet<dai::DynamicCalibrationResult>();

    // Wait a bit and ensure **no new results** arrive
    std::this_thread::sleep_for(4s);
    auto shouldBeNull = calibration_output->tryGet<dai::DynamicCalibrationResult>();
    REQUIRE(shouldBeNull == nullptr);

    REQUIRE_FALSE(sawWarnOrError);

    pipeline.stop();
    pipeline.wait();
}
