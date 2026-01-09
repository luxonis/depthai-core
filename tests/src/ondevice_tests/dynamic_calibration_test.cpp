#include <atomic>
#include <catch2/catch_all.hpp>
#include <chrono>
#include <depthai/depthai.hpp>
#include <iomanip>
#include <memory>
#include <sstream>
#include <string>
#include <thread>

// Nodes
#include "../../src/utility/Platform.hpp"
#include "depthai/common/CameraBoardSocket.hpp"
#include "depthai/pipeline/node/DynamicCalibrationNode.hpp"  // provides dai::node::DynamicCalibration
#include "depthai/utility/Compression.hpp"
#include "depthai/utility/matrixOps.hpp"

// This is a workaround to Windows API. Windows defines 2 WinAPI functions: LoadImageA and LoadImageW
// However, for user, a macro LoadImage is provided which selectes which of those 2 function will be
// used during compilation.
// When using our LoadImage in dcl namespace, the macro translates it and the compilation fails.
#undef LoadImage

using namespace std::chrono_literals;

namespace {
dai::Pipeline makePipeline(const std::shared_ptr<dai::Device>& device, std::shared_ptr<dai::node::DynamicCalibration>& dynCalib, bool linkStreams = true) {
    // Construct pipeline bound to the device
    dai::Pipeline p(device);
    dynCalib = p.create<dai::node::DynamicCalibration>();
    if(linkStreams) {
        // Cameras via .build(socket)
        auto camLeft = p.create<dai::node::Camera>()->build(dai::CameraBoardSocket::CAM_B);
        auto camRight = p.create<dai::node::Camera>()->build(dai::CameraBoardSocket::CAM_C);

        // Full-res NV12 outputs; NOTE: these return pointers
        auto* leftOut = camLeft->requestFullResolutionOutput(dai::ImgFrame::Type::NV12);
        auto* rightOut = camRight->requestFullResolutionOutput(dai::ImgFrame::Type::NV12);

        // DynamicCalibration
        leftOut->link(dynCalib->left);
        rightOut->link(dynCalib->right);
    }
    return p;
}

class TestHelper {
   public:
    TestHelper() {
        using std::filesystem::create_directories;
        using std::filesystem::path;

        testFolder = dai::platform::getTempPath();
        create_directories(testFolder);
        path outRoot = path(testFolder) / "extracted";
        create_directories(outRoot);

        auto recordingFilenames = dai::utility::filenamesInTar(RECORDING_PATH);
        std::vector<std::string> srcFiles;  // filtered names from tar (files only)
        std::vector<path> dstFiles;         // matching output paths

        srcFiles.reserve(recordingFilenames.size());
        dstFiles.reserve(recordingFilenames.size());

        for(const auto& name : recordingFilenames) {
            if(name.empty()) continue;

            // Treat entries ending with '/' (or with empty filename) as directories
            bool isDirEntry = name.back() == '/' || path{name}.filename().string().empty();
            path outPath = outRoot / name;

            if(isDirEntry) {
                // Make sure directory structure exists and skip adding to untar file lists
                create_directories(outPath);
                continue;
            }
            // Ensure parent directories exist for files
            create_directories(outPath.parent_path());

            srcFiles.push_back(name);
            dstFiles.push_back(outPath);
        }
        // Extract only file entries; directories already created above
        dai::utility::untarFiles(RECORDING_PATH, srcFiles, dstFiles);
    }

    ~TestHelper() {
        try {
            std::filesystem::remove_all(testFolder);
            std::filesystem::remove(testFolder);
        } catch(const std::exception& e) {
            std::cerr << "Failed to remove test folder: " << e.what() << std::endl;
        }
    }

    std::filesystem::path testFolder;
};

std::string makeFilename(const std::string& prefix, int index, TestHelper& helper) {
    std::ostringstream oss;
    oss << prefix << index << ".png";
    auto path = std::filesystem::path(helper.testFolder).append("extracted").append(oss.str());
    return path.string();
}

std::shared_ptr<dai::MessageGroup> stereoImageToMessageGroup(const std::string& leftPath, const std::string& rightPath) {
    // Load left image
    cv::Mat leftMat = cv::imread(leftPath, cv::IMREAD_GRAYSCALE);
    if(leftMat.empty()) {
        throw std::runtime_error("Failed to load left image: " + leftPath);
    }

    auto left = std::make_shared<dai::ImgFrame>();
    left->setType(dai::ImgFrame::Type::GRAY8);
    left->setWidth(leftMat.cols);
    left->setHeight(leftMat.rows);
    left->setCvFrame(leftMat, dai::ImgFrame::Type::GRAY8);
    left->setInstanceNum(1);

    // Load right image
    cv::Mat rightMat = cv::imread(rightPath, cv::IMREAD_GRAYSCALE);
    if(rightMat.empty()) {
        throw std::runtime_error("Failed to load right image: " + rightPath);
    }

    auto right = std::make_shared<dai::ImgFrame>();
    right->setType(dai::ImgFrame::Type::GRAY8);
    right->setWidth(rightMat.cols);
    right->setHeight(rightMat.rows);
    right->setCvFrame(rightMat, dai::ImgFrame::Type::GRAY8);
    right->setInstanceNum(2);

    auto group = std::make_shared<dai::MessageGroup>();
    group->add("right", right);
    group->add("left", left);
    return group;
}

static dai::CalibrationHandler getHandler() {
    std::vector<std::vector<float>> intrinsics = {{564.0, 0.0, 640.0}, {0.0, 564.0, 400.0}, {0.0, 0.0, 1.0}};
    std::vector<float> distortion = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

    dai::CalibrationHandler handler;
    auto eepromdata = handler.getEepromData();
    eepromdata.stereoUseSpecTranslation = false;
    eepromdata.stereoEnableDistortionCorrection = true;
    handler = dai::CalibrationHandler(eepromdata);
    handler.setCameraIntrinsics(dai::CameraBoardSocket::CAM_B, intrinsics, 1280, 800);
    handler.setCameraIntrinsics(dai::CameraBoardSocket::CAM_C, intrinsics, 1280, 800);
    handler.setDistortionCoefficients(dai::CameraBoardSocket::CAM_B, distortion);
    handler.setDistortionCoefficients(dai::CameraBoardSocket::CAM_C, distortion);
    const double rvec[3] = {0.01, 0.01, 0.01};
    std::vector<std::vector<float>> rotation = dai::matrix::rvecToRotationMatrix(rvec);
    std::vector<float> translation = {7.5, 0.0, 0.0};
    handler.setCameraExtrinsics(dai::CameraBoardSocket::CAM_B, dai::CameraBoardSocket::CAM_C, rotation, translation, {7.5f, 0.0f, 0.0f});
    return handler;
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
    auto pipeline = makePipeline(device, dynCalib);
    REQUIRE(dynCalib);

    // Queues
    auto calibrationOutput = dynCalib->calibrationOutput.createOutputQueue();
    auto coverageOutput = dynCalib->coverageOutput.createOutputQueue();
    auto commandInput = dynCalib->inputControl.createInputQueue();  // no DatatypeEnum argument

    device->setCalibration(device->readCalibration());

    pipeline.start();

    std::this_thread::sleep_for(1s);
    // Kick off calibration
    commandInput->send(std::make_shared<dai::DynamicCalibrationControl>(dai::DynamicCalibrationControl::Commands::StartCalibration{}));

    bool completed = false;
    float lastCoverage = 0.0f;
    const int kMaxIterations = 20;  // safeguard for CI/lab scenes

    for(int i = 0; i < kMaxIterations && pipeline.isRunning(); ++i) {
        // Block for coverage update and result
        auto coverage = coverageOutput->get<dai::CoverageData>();
        REQUIRE(coverage != nullptr);
        INFO("Iteration " << i << " meanCoverage=" << coverage->meanCoverage);
        if(coverage->dataAcquired < 100.0f) {
            REQUIRE(coverage->meanCoverage >= lastCoverage - 1e-4f);
        }
        lastCoverage = coverage->meanCoverage;

        auto result = calibrationOutput->get<dai::DynamicCalibrationResult>();
        REQUIRE(result != nullptr);

        if(result->calibrationData.has_value()) {
            completed = true;

            commandInput->send(std::make_shared<dai::DynamicCalibrationControl>(
                dai::DynamicCalibrationControl::Commands::ApplyCalibration{result->calibrationData->newCalibration}));
            break;
        } else {
            REQUIRE(!result->info.empty());
        }
    }

    if(lastCoverage < 100.0f) {
        // If the coverage is lower then requested, try to force calibrate it.
        auto qcmd = std::make_shared<dai::DynamicCalibrationControl>(dai::DynamicCalibrationControl::Commands::Calibrate{true});
        commandInput->send(qcmd);

        auto result = calibrationOutput->get<dai::DynamicCalibrationResult>();

        // If there will be enough data, the result should have value and it should be calibrationData
        if(result->calibrationData.has_value()) {
            // We expect to see a payload only when the process is complete
            completed = true;

            // Optional: immediately apply it (like your example)
            commandInput->send(std::make_shared<dai::DynamicCalibrationControl>(
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
    auto pipeline = makePipeline(device, dynCalib);
    REQUIRE(dynCalib);

    auto calibrationOutput = dynCalib->calibrationOutput.createOutputQueue();
    auto qualityOutput = dynCalib->qualityOutput.createOutputQueue();
    auto commandInput = dynCalib->inputControl.createInputQueue();  // no DatatypeEnum argument

    device->setCalibration(device->readCalibration());

    pipeline.start();
    std::this_thread::sleep_for(1s);

    // 1) Calibrate (default)
    {
        commandInput->send(std::make_shared<dai::DynamicCalibrationControl>(dai::DynamicCalibrationControl::Commands::Calibrate{}));
        auto result = calibrationOutput->get<dai::DynamicCalibrationResult>();
        REQUIRE(result != nullptr);
        INFO("Calibrate #1 info: " << result->info);
        REQUIRE_FALSE(result->calibrationData.has_value());
    }

    // 2) Calibrate(force=true)
    {
        commandInput->send(std::make_shared<dai::DynamicCalibrationControl>(dai::DynamicCalibrationControl::Commands::Calibrate{true}));
        auto result = calibrationOutput->get<dai::DynamicCalibrationResult>();
        REQUIRE(result != nullptr);
        INFO("Calibrate #2 (force) info: " << result->info);
        REQUIRE_FALSE(result->calibrationData.has_value());
    }

    // 3) CalibrationQuality(force=true)
    {
        commandInput->send(std::make_shared<dai::DynamicCalibrationControl>(dai::DynamicCalibrationControl::Commands::CalibrationQuality{true}));
        auto qres = qualityOutput->get<dai::CalibrationQuality>();
        REQUIRE(qres != nullptr);
        INFO("Quality #1 (force) info: " << qres->info);
        REQUIRE_FALSE(qres->qualityData.has_value());
    }

    // 4) CalibrationQuality(force=false)
    {
        commandInput->send(std::make_shared<dai::DynamicCalibrationControl>(dai::DynamicCalibrationControl::Commands::CalibrationQuality{false}));
        auto qres = qualityOutput->get<dai::CalibrationQuality>();
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
    auto pipeline = makePipeline(device, dynCalib);
    REQUIRE(dynCalib);

    auto calibrationOutput = dynCalib->calibrationOutput.createOutputQueue();
    auto commandInput = dynCalib->inputControl.createInputQueue();  // no DatatypeEnum argument

    device->setCalibration(device->readCalibration());

    pipeline.start();

    commandInput->send(std::make_shared<dai::DynamicCalibrationControl>(dai::DynamicCalibrationControl::Commands::StartCalibration{}));

    auto first = calibrationOutput->get<dai::DynamicCalibrationResult>();
    REQUIRE(first != nullptr);

    // Stop
    commandInput->send(std::make_shared<dai::DynamicCalibrationControl>(dai::DynamicCalibrationControl::Commands::StopCalibration{}));

    (void)calibrationOutput->tryGet<dai::DynamicCalibrationResult>();  // drain in-flight if any

    std::this_thread::sleep_for(4s);
    auto shouldBeNull = calibrationOutput->tryGet<dai::DynamicCalibrationResult>();
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
    auto pipeline = makePipeline(device, dynCalib);
    REQUIRE(dynCalib);

    auto calibrationOutput = dynCalib->calibrationOutput.createOutputQueue();
    auto coverageOutput = dynCalib->coverageOutput.createOutputQueue();
    auto commandInput = dynCalib->inputControl.createInputQueue();  // no DatatypeEnum argument

    device->setCalibration(device->readCalibration());

    pipeline.start();
    std::this_thread::sleep_for(1s);

    // Load one image into the calibration process to produce coverage
    commandInput->send(std::make_shared<dai::DynamicCalibrationControl>(dai::DynamicCalibrationControl::Commands::LoadImage{}));
    (void)coverageOutput->get<dai::CoverageData>();

    // Reset
    commandInput->send(std::make_shared<dai::DynamicCalibrationControl>(dai::DynamicCalibrationControl::Commands::ResetData{}));

    // Force calibrate; expect no calibrationData due to empty accumulators
    commandInput->send(std::make_shared<dai::DynamicCalibrationControl>(dai::DynamicCalibrationControl::Commands::Calibrate{true}));
    auto result = calibrationOutput->get<dai::DynamicCalibrationResult>();
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
    auto pipeline = makePipeline(device, dynCalib);
    REQUIRE(dynCalib);

    auto commandInput = dynCalib->inputControl.createInputQueue();  // no DatatypeEnum argument

    device->setCalibration(device->readCalibration());

    pipeline.start();

    commandInput->send(std::make_shared<dai::DynamicCalibrationControl>());
    std::this_thread::sleep_for(0.5s);

    pipeline.stop();
    pipeline.wait();
}

TEST_CASE("DynamicCalibration: Recalibration on synthetic data.") {
    TestHelper helper;

    auto device = std::make_shared<dai::Device>();
    std::shared_ptr<dai::node::DynamicCalibration> dynCalib;
    auto p = makePipeline(device, dynCalib, false);

    auto coverageOutput = dynCalib->coverageOutput.createOutputQueue();
    auto commandInput = dynCalib->inputControl.createInputQueue();
    auto calibrationOutput = dynCalib->calibrationOutput.createOutputQueue();

    device->setCalibration(getHandler());
    auto group = stereoImageToMessageGroup(makeFilename("data/LeftCam_", 0, helper), makeFilename("data/RightCam_", 0, helper));
    p.start();
    dynCalib->syncInput.send(group);
    std::this_thread::sleep_for(0.5s);

    // load image
    for(int i = 0; i < 7; i++) {
        auto group = stereoImageToMessageGroup(makeFilename("data/LeftCam_", i, helper), makeFilename("data/RightCam_", i, helper));
        dynCalib->syncInput.send(group);
        commandInput->send(std::make_shared<dai::DynamicCalibrationControl>(dai::DynamicCalibrationControl::Commands::LoadImage{}));
        auto coverage = coverageOutput->get<dai::CoverageData>();
        REQUIRE(coverage->coverageAcquired > 0.0f);
    }

    // calibrate
    commandInput->send(std::make_shared<dai::DynamicCalibrationControl>(dai::DynamicCalibrationControl::Commands::Calibrate{true}));
    auto result = calibrationOutput->get<dai::DynamicCalibrationResult>();

    REQUIRE(result != nullptr);
    REQUIRE(result->calibrationData != std::nullopt);

    auto rotationMatrixOld = result->calibrationData->currentCalibration.getCameraRotationMatrix(dai::CameraBoardSocket::CAM_B, dai::CameraBoardSocket::CAM_C);
    std::vector<float> rvecOld = dai::matrix::rotationMatrixToVector(rotationMatrixOld);
    auto rotationMatrix = result->calibrationData->newCalibration.getCameraRotationMatrix(dai::CameraBoardSocket::CAM_B, dai::CameraBoardSocket::CAM_C);
    REQUIRE(std::fabs(rvecOld[0] - 0.01) < 0.00001);
    REQUIRE(std::fabs(rvecOld[1] - 0.01) < 0.00001);
    REQUIRE(std::fabs(rvecOld[2] - 0.01) < 0.00001);

    std::vector<float> rvec = dai::matrix::rotationMatrixToVector(rotationMatrix);
    float threshold = 1e-7f;
    REQUIRE(std::fabs(rvec[0]) < threshold);
    REQUIRE(std::fabs(rvec[1]) < threshold);
    REQUIRE(std::fabs(rvec[2]) < threshold);
    p.stop();
    p.wait();
}
