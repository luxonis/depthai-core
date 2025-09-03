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
    auto device = std::make_shared<dai::Device>();
    REQUIRE(device != nullptr);

    std::atomic<bool> sawWarnOrError{false};
    device->setLogLevel(dai::LogLevel::WARN);
    device->addLogCallback([&](const dai::LogMessage& m) {
        if(m.level >= dai::LogLevel::WARN) sawWarnOrError = true;
    });

    std::shared_ptr<dai::node::DynamicCalibration> dynCalib;
    std::shared_ptr<dai::node::StereoDepth> stereo;
    auto pipeline = makePipeline(device, dynCalib, stereo);
    REQUIRE(dynCalib);
    REQUIRE(stereo);

    // Queues
    auto calibration_output = dynCalib->calibrationOutput.createOutputQueue();
    auto coverage_output = dynCalib->coverageOutput.createOutputQueue();
    auto command_input = dynCalib->inputControl.createInputQueue();  // no DatatypeEnum argument

    auto left_xout = stereo->syncedLeft.createOutputQueue();
    auto right_xout = stereo->syncedRight.createOutputQueue();
    auto disp_xout = stereo->disparity.createOutputQueue();

    device->setCalibration(device->readCalibration());

    pipeline.start();

    std::this_thread::sleep_for(1s);

    {
        auto l = left_xout->get<dai::ImgFrame>();
        auto r = right_xout->get<dai::ImgFrame>();
        auto d = disp_xout->get<dai::ImgFrame>();
        REQUIRE(l != nullptr);
        REQUIRE(r != nullptr);
        REQUIRE(d != nullptr);
    }

    // Kick off calibration
    command_input->send(std::make_shared<dai::DynamicCalibrationControl>(dai::DynamicCalibrationControl::Commands::StartCalibration{}));

    bool completed = false;
    float lastCoverage = 0.0f;
    const int kMaxIterations = 20;  // safeguard for CI/lab scenes

    for(int i = 0; i < kMaxIterations && pipeline.isRunning(); ++i) {
        // Block for coverage update and result
        auto coverage = coverage_output->get<dai::CoverageData>();
        REQUIRE(coverage != nullptr);
        INFO("Iteration " << i << " meanCoverage=" << coverage->meanCoverage);
        if(coverage->dataAcquired < 100.0f) {
            REQUIRE(coverage->meanCoverage >= lastCoverage - 1e-4f);
        }
        lastCoverage = coverage->meanCoverage;

        auto result = calibration_output->get<dai::DynamicCalibrationResult>();
        REQUIRE(result != nullptr);

        if(result->calibrationData.has_value()) {
            completed = true;

            command_input->send(std::make_shared<dai::DynamicCalibrationControl>(
                dai::DynamicCalibrationControl::Commands::ApplyCalibration{result->calibrationData->newCalibration}));
            break;
        } else {
            REQUIRE(!result->info.empty());
        }
    }

    if(lastCoverage < 100.0f) {
        // If the coverage is lower then requested, try to force calibrate it.
        auto qcmd = std::make_shared<dai::DynamicCalibrationControl>(dai::DynamicCalibrationControl::Commands::Calibrate{true});
        command_input->send(qcmd);

        auto result = calibration_output->get<dai::DynamicCalibrationResult>();

        // If there will be enough data, the result should have value and it should be calibrationData
        if(result->calibrationData.has_value()) {
            // We expect to see a payload only when the process is complete
            completed = true;

            // Optional: immediately apply it (like your example)
            command_input->send(std::make_shared<dai::DynamicCalibrationControl>(
                dai::DynamicCalibrationControl::Commands::ApplyCalibration{result->calibrationData->newCalibration}));
        } else {
            // While running, info should be non-empty (typically progress/status text)
            REQUIRE(!result->info.empty());
            REQUIRE(!result->calibrationData.has_value());
            completed = true;
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

    std::atomic<bool> sawWarnOrError{false};
    device->setLogLevel(dai::LogLevel::WARN);
    device->addLogCallback([&](const dai::LogMessage& m) {
        if(m.level >= dai::LogLevel::WARN) sawWarnOrError = true;
    });

    std::shared_ptr<dai::node::DynamicCalibration> dynCalib;
    std::shared_ptr<dai::node::StereoDepth> stereo;
    auto pipeline = makePipeline(device, dynCalib, stereo);
    REQUIRE(dynCalib);
    REQUIRE(stereo);

    auto calibration_output = dynCalib->calibrationOutput.createOutputQueue();
    auto quality_output = dynCalib->qualityOutput.createOutputQueue();
    auto command_input = dynCalib->inputControl.createInputQueue();  // no DatatypeEnum argument

    auto left_xout = stereo->syncedLeft.createOutputQueue();
    auto right_xout = stereo->syncedRight.createOutputQueue();
    auto disp_xout = stereo->disparity.createOutputQueue();

    device->setCalibration(device->readCalibration());

    pipeline.start();
    std::this_thread::sleep_for(1s);

    REQUIRE(left_xout->get<dai::ImgFrame>() != nullptr);
    REQUIRE(right_xout->get<dai::ImgFrame>() != nullptr);
    REQUIRE(disp_xout->get<dai::ImgFrame>() != nullptr);

    // 1) Calibrate (default)
    {
        command_input->send(std::make_shared<dai::DynamicCalibrationControl>(dai::DynamicCalibrationControl::Commands::Calibrate{}));
        auto result = calibration_output->get<dai::DynamicCalibrationResult>();
        REQUIRE(result != nullptr);
        INFO("Calibrate #1 info: " << result->info);
        REQUIRE_FALSE(result->calibrationData.has_value());
    }

    // 2) Calibrate(force=true)
    {
        command_input->send(std::make_shared<dai::DynamicCalibrationControl>(dai::DynamicCalibrationControl::Commands::Calibrate{true}));
        auto result = calibration_output->get<dai::DynamicCalibrationResult>();
        REQUIRE(result != nullptr);
        INFO("Calibrate #2 (force) info: " << result->info);
        REQUIRE_FALSE(result->calibrationData.has_value());
    }

    // 3) CalibrationQuality(force=true)
    {
        command_input->send(std::make_shared<dai::DynamicCalibrationControl>(dai::DynamicCalibrationControl::Commands::CalibrationQuality{true}));
        auto qres = quality_output->get<dai::CalibrationQuality>();
        REQUIRE(qres != nullptr);
        INFO("Quality #1 (force) info: " << qres->info);
        REQUIRE_FALSE(qres->qualityData.has_value());
    }

    // 4) CalibrationQuality(force=false)
    {
        command_input->send(std::make_shared<dai::DynamicCalibrationControl>(dai::DynamicCalibrationControl::Commands::CalibrationQuality{false}));
        auto qres = quality_output->get<dai::CalibrationQuality>();
        REQUIRE(qres != nullptr);
        INFO("Quality #2 (no force) info: " << qres->info);
        REQUIRE_FALSE(qres->qualityData.has_value());
    }

    REQUIRE_FALSE(sawWarnOrError);

    pipeline.stop();
    pipeline.wait();
}

TEST_CASE("DynamicCalibration: StopCalibration halts further results") {
    auto device = std::make_shared<dai::Device>();
    REQUIRE(device != nullptr);

    std::atomic<bool> sawWarnOrError{false};
    device->setLogLevel(dai::LogLevel::WARN);
    device->addLogCallback([&](const dai::LogMessage& m) {
        if(m.level >= dai::LogLevel::WARN) sawWarnOrError = true;
    });

    std::shared_ptr<dai::node::DynamicCalibration> dynCalib;
    std::shared_ptr<dai::node::StereoDepth> stereo;
    auto pipeline = makePipeline(device, dynCalib, stereo);
    REQUIRE(dynCalib);
    REQUIRE(stereo);

    auto calibration_output = dynCalib->calibrationOutput.createOutputQueue();
    auto command_input = dynCalib->inputControl.createInputQueue();  // no DatatypeEnum argument

    auto left_xout = stereo->syncedLeft.createOutputQueue();
    auto right_xout = stereo->syncedRight.createOutputQueue();
    auto disp_xout = stereo->disparity.createOutputQueue();

    device->setCalibration(device->readCalibration());

    pipeline.start();

    REQUIRE(left_xout->get<dai::ImgFrame>() != nullptr);
    REQUIRE(right_xout->get<dai::ImgFrame>() != nullptr);
    REQUIRE(disp_xout->get<dai::ImgFrame>() != nullptr);

    command_input->send(std::make_shared<dai::DynamicCalibrationControl>(dai::DynamicCalibrationControl::Commands::StartCalibration{}));

    auto first = calibration_output->get<dai::DynamicCalibrationResult>();
    REQUIRE(first != nullptr);

    // Stop
    command_input->send(std::make_shared<dai::DynamicCalibrationControl>(dai::DynamicCalibrationControl::Commands::StopCalibration{}));

    (void)calibration_output->tryGet<dai::DynamicCalibrationResult>();  // drain in-flight if any

    std::this_thread::sleep_for(4s);
    auto shouldBeNull = calibration_output->tryGet<dai::DynamicCalibrationResult>();
    REQUIRE(shouldBeNull == nullptr);

    REQUIRE_FALSE(sawWarnOrError);

    pipeline.stop();
    pipeline.wait();
}

TEST_CASE("DynamicCalibration: reset data") {
    auto device = std::make_shared<dai::Device>();
    REQUIRE(device != nullptr);

    std::atomic<bool> sawWarnOrError{false};
    device->setLogLevel(dai::LogLevel::WARN);
    device->addLogCallback([&](const dai::LogMessage& m) {
        if(m.level >= dai::LogLevel::WARN) sawWarnOrError = true;
    });

    std::shared_ptr<dai::node::DynamicCalibration> dynCalib;
    std::shared_ptr<dai::node::StereoDepth> stereo;
    auto pipeline = makePipeline(device, dynCalib, stereo);
    REQUIRE(dynCalib);
    REQUIRE(stereo);

    auto calibration_output = dynCalib->calibrationOutput.createOutputQueue();
    auto coverage_output = dynCalib->coverageOutput.createOutputQueue();
    auto command_input = dynCalib->inputControl.createInputQueue();  // no DatatypeEnum argument

    auto left_xout = stereo->syncedLeft.createOutputQueue();
    auto right_xout = stereo->syncedRight.createOutputQueue();
    auto disp_xout = stereo->disparity.createOutputQueue();

    device->setCalibration(device->readCalibration());

    pipeline.start();
    std::this_thread::sleep_for(1s);

    REQUIRE(left_xout->get<dai::ImgFrame>() != nullptr);
    REQUIRE(right_xout->get<dai::ImgFrame>() != nullptr);
    REQUIRE(disp_xout->get<dai::ImgFrame>() != nullptr);

    // Load one image into the calibration process to produce coverage
    command_input->send(std::make_shared<dai::DynamicCalibrationControl>(dai::DynamicCalibrationControl::Commands::LoadImage{}));
    (void)coverage_output->get<dai::CoverageData>();

    // Reset
    command_input->send(std::make_shared<dai::DynamicCalibrationControl>(dai::DynamicCalibrationControl::Commands::ResetData{}));

    // Force calibrate; expect no calibrationData due to empty accumulators
    command_input->send(std::make_shared<dai::DynamicCalibrationControl>(dai::DynamicCalibrationControl::Commands::Calibrate{true}));
    auto result = calibration_output->get<dai::DynamicCalibrationResult>();
    REQUIRE(result != nullptr);
    REQUIRE(result->calibrationData == std::nullopt);

    REQUIRE_FALSE(sawWarnOrError);

    pipeline.stop();
    pipeline.wait();
}

TEST_CASE("DynamicCalibration: Empty command") {
    auto device = std::make_shared<dai::Device>();
    REQUIRE(device != nullptr);

    std::atomic<bool> sawWarnOrError{false};
    device->setLogLevel(dai::LogLevel::WARN);
    device->addLogCallback([&](const dai::LogMessage& m) {
        if(m.level >= dai::LogLevel::WARN) sawWarnOrError = true;
    });

    std::shared_ptr<dai::node::DynamicCalibration> dynCalib;
    std::shared_ptr<dai::node::StereoDepth> stereo;
    auto pipeline = makePipeline(device, dynCalib, stereo);
    REQUIRE(dynCalib);

    auto command_input = dynCalib->inputControl.createInputQueue();  // no DatatypeEnum argument

    auto left_xout = stereo->syncedLeft.createOutputQueue();
    auto right_xout = stereo->syncedRight.createOutputQueue();
    auto disp_xout = stereo->disparity.createOutputQueue();

    device->setCalibration(device->readCalibration());

    pipeline.start();

    command_input->send(std::make_shared<dai::DynamicCalibrationControl>());
    std::this_thread::sleep_for(0.5s);

    pipeline.stop();
    pipeline.wait();
}
