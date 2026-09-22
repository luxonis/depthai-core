#include <array>
#include <atomic>
#include <catch2/catch_all.hpp>
#include <chrono>
#include <cmath>
#include <depthai/depthai.hpp>
#include <exception>
#include <iomanip>
#include <memory>
#include <optional>
#include <sstream>
#include <string>
#include <thread>
#include <utility>

// Nodes
#include "../../src/pipeline/node/DynamicCalibrationTransforms.hpp"
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
constexpr int HOLISTIC_REPLAY_IMAGE_COUNT = 10;
constexpr auto HOLISTIC_REPLAY_FRAME_INTERVAL = 150ms;

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

        auto recordingFilenames = dai::utility::filenamesInArchive(RECORDING_PATH);
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
        dai::utility::extractFiles(RECORDING_PATH, srcFiles, dstFiles);
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

std::array<std::array<float, 3>, 3> toArray3x3(const std::vector<std::vector<float>>& matrix) {
    std::array<std::array<float, 3>, 3> out{};
    for(size_t i = 0; i < 3; ++i) {
        for(size_t j = 0; j < 3; ++j) {
            out[i][j] = matrix.at(i).at(j);
        }
    }
    return out;
}

void setFrameTransformation(const std::shared_ptr<dai::ImgFrame>& frame, const dai::CalibrationHandler& handler, dai::CameraBoardSocket socket) {
    auto intrinsics = handler.getCameraIntrinsics(socket, frame->getWidth(), frame->getHeight());
    auto distortion = handler.getDistortionCoefficients(socket);
    auto distortionModel = handler.getDistortionModel(socket);
    frame->setTransformation(dai::ImgTransformation(frame->getWidth(), frame->getHeight(), toArray3x3(intrinsics), distortionModel, distortion));
}

std::shared_ptr<dai::MessageGroup> stereoImageToMessageGroup(const std::string& leftPath,
                                                             const std::string& rightPath,
                                                             const dai::CalibrationHandler& handler) {
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
    setFrameTransformation(left, handler, dai::CameraBoardSocket::CAM_B);

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
    setFrameTransformation(right, handler, dai::CameraBoardSocket::CAM_C);

    auto group = std::make_shared<dai::MessageGroup>();
    group->add("right", right);
    group->add("left", left);
    return group;
}

static dai::CalibrationHandler getHandler(bool toHousing = false) {
    auto camLeft = dai::CameraBoardSocket::CAM_C;
    auto camRight = dai::CameraBoardSocket::CAM_B;
    auto camBase = dai::CameraBoardSocket::CAM_A;

    const double rvecLeftToRight[3] = {0.01, 0.01, 0.01};
    std::vector<std::vector<float>> rotationLeftToRight = dai::matrix::rvecToRotationMatrix(rvecLeftToRight);
    std::vector<float> cvecLeftToRight = {-7.5, 0.0, 0.0};
    auto tvecLeftToRight = dai::matrix::matVecMul(rotationLeftToRight, cvecLeftToRight);

    const double rvecRightToBase[3] = {0.0, 0.0, 0.1};
    std::vector<std::vector<float>> rotationRightToBase = dai::matrix::rvecToRotationMatrix(rvecRightToBase);
    std::vector<float> cvecRightToBase = {4.5f, 0.0f, 0.0f};
    auto tvecRightToBase = dai::matrix::matVecMul(rotationRightToBase, cvecRightToBase);
    auto [rotationBaseToRight, tvecBaseToRight] = dai::matrix::invertSe3(rotationRightToBase, tvecRightToBase);

    std::vector<std::vector<float>> intrinsics = {{564.0, 0.0, 640.0}, {0.0, 564.0, 400.0}, {0.0, 0.0, 1.0}};
    std::vector<float> distortion = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    std::vector<std::vector<float>> identity = {{1.0f, 0.0f, 0.0f}, {0.0f, 1.0f, 0.0f}, {0.0f, 0.0f, 1.0f}};

    dai::CalibrationHandler handler;
    auto eepromdata = handler.getEepromData();
    eepromdata.stereoUseSpecTranslation = false;
    eepromdata.stereoEnableDistortionCorrection = true;
    eepromdata.housingExtrinsics.toCameraSocket = toHousing ? camRight : camBase;
    eepromdata.housingExtrinsics.rotationMatrix = toHousing ? rotationBaseToRight : identity;
    eepromdata.housingExtrinsics.translation =
        toHousing ? dai::Point3f(tvecBaseToRight[0], tvecBaseToRight[1], tvecBaseToRight[2]) : dai::Point3f(0.0f, 0.0f, 0.0f);
    handler = dai::CalibrationHandler(eepromdata);
    handler.setCameraIntrinsics(camLeft, intrinsics, 1280, 800);
    handler.setCameraIntrinsics(camRight, intrinsics, 1280, 800);
    handler.setDistortionCoefficients(camLeft, distortion);
    handler.setDistortionCoefficients(camRight, distortion);

    handler.setCameraExtrinsics(camLeft, camRight, rotationLeftToRight, tvecLeftToRight, cvecLeftToRight);

    if(!toHousing) {
        handler.setCameraIntrinsics(camBase, intrinsics, 1280, 800);
        handler.setDistortionCoefficients(camBase, distortion);
        handler.setCameraExtrinsics(camRight, camBase, rotationRightToBase, tvecRightToBase, cvecRightToBase);
    }

    return handler;
}

static dai::CalibrationHandler getTwoCameraHandler(const std::array<double, 3>& rvecLeftToRight, const std::vector<float>& cvecLeftToRight) {
    const auto camLeft = dai::CameraBoardSocket::CAM_C;
    const auto camRight = dai::CameraBoardSocket::CAM_B;
    const auto rotationLeftToRight = dai::matrix::rvecToRotationMatrix(rvecLeftToRight.data());
    const auto tvecLeftToRight = dai::matrix::matVecMul(rotationLeftToRight, cvecLeftToRight);
    const std::vector<std::vector<float>> intrinsics = {{564.0f, 0.0f, 640.0f}, {0.0f, 564.0f, 400.0f}, {0.0f, 0.0f, 1.0f}};
    const std::vector<float> distortion(14, 0.0f);

    dai::CalibrationHandler handler;
    auto eepromData = handler.getEepromData();
    eepromData.stereoUseSpecTranslation = false;
    eepromData.stereoEnableDistortionCorrection = true;
    handler = dai::CalibrationHandler(eepromData);
    handler.setCameraIntrinsics(camLeft, intrinsics, 1280, 800);
    handler.setCameraIntrinsics(camRight, intrinsics, 1280, 800);
    handler.setDistortionCoefficients(camLeft, distortion);
    handler.setDistortionCoefficients(camRight, distortion);
    handler.setCameraExtrinsics(camLeft, camRight, rotationLeftToRight, tvecLeftToRight, cvecLeftToRight);
    return handler;
}

std::vector<float> rotationDifferenceVector(const std::vector<std::vector<float>>& rotation, const std::vector<std::vector<float>>& referenceRotation) {
    const auto referenceRotationInverse = dai::matrix::invertSe3(referenceRotation, {0.0f, 0.0f, 0.0f}).first;
    return dai::matrix::rotationMatrixToVector(dai::matrix::matMul(rotation, referenceRotationInverse));
}

struct HolisticCalibrationSetup {
    dai::CalibrationHandler perturbedCalibration;
    std::vector<std::vector<float>> groundTruthRotation;
    std::vector<std::vector<float>> perturbedRotation;
};

HolisticCalibrationSetup makeHolisticCalibrationSetup(const dai::CalibrationHandler& replayCalibration) {
    auto groundTruthEeprom = replayCalibration.getEepromData();
    // This recording intentionally contains a synthetic housing transform to CAM_E
    // for transformation tests. Dynamic calibration only needs a valid housing root.
    groundTruthEeprom.housingExtrinsics.toCameraSocket = dai::CameraBoardSocket::CAM_A;
    const dai::CalibrationHandler groundTruthCalibration(groundTruthEeprom);

    constexpr auto CAM_LEFT = dai::CameraBoardSocket::CAM_B;
    constexpr auto CAM_RIGHT = dai::CameraBoardSocket::CAM_C;
    const auto groundTruthRotation = groundTruthCalibration.getCameraRotationMatrix(CAM_LEFT, CAM_RIGHT);
    const auto groundTruthRvec = dai::matrix::rotationMatrixToVector(groundTruthRotation);
    const std::array<double, 3> perturbedRvec = {groundTruthRvec[0] + 0.01, groundTruthRvec[1] + 0.01, groundTruthRvec[2] + 0.01};
    const auto perturbedRotation = dai::matrix::rvecToRotationMatrix(perturbedRvec.data());
    const auto cameraCenter = groundTruthCalibration.getCameraTranslationVector(CAM_LEFT, CAM_RIGHT, false);
    const auto perturbedTranslation = dai::matrix::matVecMul(perturbedRotation, cameraCenter);

    auto perturbedEeprom = groundTruthEeprom;
    auto& perturbedExtrinsics = perturbedEeprom.cameraData.at(CAM_LEFT).extrinsics;
    perturbedExtrinsics.rotationMatrix = perturbedRotation;
    perturbedExtrinsics.translation = dai::Point3f(perturbedTranslation[0], perturbedTranslation[1], perturbedTranslation[2]);
    return {dai::CalibrationHandler(perturbedEeprom), groundTruthRotation, perturbedRotation};
}

struct HolisticRecalibrationObservation {
    std::array<std::optional<float>, HOLISTIC_REPLAY_IMAGE_COUNT> coverageByImage;
    std::shared_ptr<dai::DynamicCalibrationResult> result;
    std::vector<std::vector<float>> groundTruthRotation;
    std::vector<std::vector<float>> perturbedRotation;
    // CAM_A is not a calibration input. The node places it through its factory transform from the anchor camera:
    // the default stereo-depth reference of the first device stereo pair whose reference is a calibration input.
    std::optional<dai::CalibrationHandler> factoryCalibration;
    std::optional<dai::CameraBoardSocket> anchor;
};

HolisticRecalibrationObservation runHolisticRecalibrationAttempt() {
    constexpr auto CAM_LEFT = dai::CameraBoardSocket::CAM_B;
    constexpr auto CAM_RIGHT = dai::CameraBoardSocket::CAM_C;
    auto device = std::make_shared<dai::Device>();
    dai::Pipeline pipeline(device);
    pipeline.enableHolisticReplay(HOLISTIC_RECORDING_PATH);
    const auto setup = makeHolisticCalibrationSetup(device->getCalibration());
    device->setCalibration(setup.perturbedCalibration);

    auto camLeft = pipeline.create<dai::node::Camera>()->build(CAM_LEFT);
    auto camRight = pipeline.create<dai::node::Camera>()->build(CAM_RIGHT);
    auto* leftOut = camLeft->requestFullResolutionOutput(dai::ImgFrame::Type::NV12);
    auto* rightOut = camRight->requestFullResolutionOutput(dai::ImgFrame::Type::NV12);
    auto dynCalib = pipeline.create<dai::node::DynamicCalibration>();
    leftOut->link(dynCalib->left);
    rightOut->link(dynCalib->right);

    auto coverageOutput = dynCalib->coverageOutput.createOutputQueue();
    auto commandInput = dynCalib->inputControl.createInputQueue();
    auto calibrationOutput = dynCalib->calibrationOutput.createOutputQueue();
    HolisticRecalibrationObservation observation;
    observation.groundTruthRotation = setup.groundTruthRotation;
    observation.perturbedRotation = setup.perturbedRotation;
    try {
        observation.factoryCalibration = device->readFactoryCalibration();
    } catch(const std::exception&) {
        // Reported by the test case: the node cannot anchor CAM_A without factory calibration.
    }
    for(const auto& stereoPair : device->getStereoPairs()) {
        const auto candidate = dai::node::detail::stereoDepthReferenceCamera(stereoPair, setup.perturbedCalibration, device->getPlatform());
        if(candidate == CAM_LEFT || candidate == CAM_RIGHT) {
            observation.anchor = candidate;
            break;
        }
    }

    pipeline.start();
    std::this_thread::sleep_for(0.5s);
    for(int imageIndex = 0; imageIndex < HOLISTIC_REPLAY_IMAGE_COUNT; ++imageIndex) {
        commandInput->send(std::make_shared<dai::DynamicCalibrationControl>(dai::DynamicCalibrationControl::Commands::LoadImage{}));
        const auto coverage = coverageOutput->get<dai::CoverageData>();
        if(coverage != nullptr) observation.coverageByImage[imageIndex] = coverage->coverageAcquired;
        std::this_thread::sleep_for(HOLISTIC_REPLAY_FRAME_INTERVAL);
    }
    commandInput->send(std::make_shared<dai::DynamicCalibrationControl>(dai::DynamicCalibrationControl::Commands::Calibrate{true}));
    observation.result = calibrationOutput->get<dai::DynamicCalibrationResult>();
    pipeline.stop();
    pipeline.wait();
    return observation;
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

    device->setCalibration(device->getCalibration());

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

    device->setCalibration(device->getCalibration());

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

    device->setCalibration(device->getCalibration());

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

    device->setCalibration(device->getCalibration());

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

    device->setCalibration(device->getCalibration());

    pipeline.start();

    commandInput->send(std::make_shared<dai::DynamicCalibrationControl>());
    std::this_thread::sleep_for(0.5s);

    pipeline.stop();
    pipeline.wait();
}

TEST_CASE("DynamicCalibration: Recalibration on holistic replay data.") {
    constexpr auto CAM_LEFT = dai::CameraBoardSocket::CAM_B;
    constexpr auto CAM_RIGHT = dai::CameraBoardSocket::CAM_C;
    const auto observation = runHolisticRecalibrationAttempt();
    for(const auto coverage : observation.coverageByImage) {
        REQUIRE(coverage.has_value());
        REQUIRE(*coverage > 0.0f);
    }
    if(!observation.factoryCalibration.has_value()) SKIP("Device has no factory calibration to anchor CAM_A");
    REQUIRE(observation.anchor.has_value());
    REQUIRE(observation.result != nullptr);
    REQUIRE(observation.result->calibrationData.has_value());

    const auto& calibrationData = *observation.result->calibrationData;
    const auto currentRotation = calibrationData.currentCalibration.getCameraRotationMatrix(CAM_LEFT, CAM_RIGHT);
    const auto newRotation = calibrationData.newCalibration.getCameraRotationMatrix(CAM_LEFT, CAM_RIGHT);
    const auto currentRotationError = rotationDifferenceVector(currentRotation, observation.perturbedRotation);
    const auto newRotationError = rotationDifferenceVector(newRotation, observation.groundTruthRotation);
    // The unobserved CAM_A keeps its factory transform from the anchor camera.
    const auto anchor = *observation.anchor;
    const auto anchorToBase = calibrationData.newCalibration.getCameraExtrinsics(anchor, dai::CameraBoardSocket::CAM_A, false);
    const auto anchorToBaseFactory = observation.factoryCalibration->getCameraExtrinsics(anchor, dai::CameraBoardSocket::CAM_A, false);

    REQUIRE(currentRotationError.size() >= 3);
    REQUIRE(newRotationError.size() >= 3);

    constexpr float CURRENT_ROTATION_THRESHOLD = 1e-5f;
    constexpr float NEW_ROTATION_THRESHOLD = 1.2e-3f;
    constexpr float FACTORY_ROTATION_MATRIX_THRESHOLD = 1e-5f;
    constexpr float FACTORY_TRANSLATION_THRESHOLD = 1e-4f;  // centimeters
    CAPTURE(anchor, newRotationError[0], newRotationError[1], newRotationError[2]);
    for(std::size_t axis = 0; axis < 3; ++axis) {
        REQUIRE(std::fabs(currentRotationError[axis]) < CURRENT_ROTATION_THRESHOLD);
        REQUIRE(std::fabs(newRotationError[axis]) < NEW_ROTATION_THRESHOLD);
    }
    for(std::size_t row = 0; row < 3; ++row) {
        for(std::size_t col = 0; col < 3; ++col) {
            REQUIRE(std::fabs(anchorToBase[row][col] - anchorToBaseFactory[row][col]) < FACTORY_ROTATION_MATRIX_THRESHOLD);
        }
        REQUIRE(std::fabs(anchorToBase[row][3] - anchorToBaseFactory[row][3]) < FACTORY_TRANSLATION_THRESHOLD);
    }
    REQUIRE(calibrationData.calibrationDifference.sampsonErrorNew < calibrationData.calibrationDifference.sampsonErrorCurrent);
}

TEST_CASE("DynamicCalibration: Rejects excessive translation direction change.") {
    TestHelper helper;

    auto device = std::make_shared<dai::Device>();
    std::shared_ptr<dai::node::DynamicCalibration> dynCalib;
    auto p = makePipeline(device, dynCalib, false);

    auto coverageOutput = dynCalib->coverageOutput.createOutputQueue();
    auto commandInput = dynCalib->inputControl.createInputQueue();
    auto calibrationOutput = dynCalib->calibrationOutput.createOutputQueue();

    // Give DCL a baseline direction which differs substantially from the
    // direction implied by the recorded stereo images.
    auto calibrationHandler = getTwoCameraHandler({0.0, 0.0, 0.0}, {-5.0f, -5.0f, 0.0f});
    device->setCalibration(calibrationHandler);

    auto group = stereoImageToMessageGroup(makeFilename("data/LeftCam_", 0, helper), makeFilename("data/RightCam_", 0, helper), calibrationHandler);
    p.start();
    dynCalib->syncInput.send(group);
    std::this_thread::sleep_for(0.5s);

    for(int i = 0; i < 7; ++i) {
        auto frameGroup = stereoImageToMessageGroup(makeFilename("data/LeftCam_", i, helper), makeFilename("data/RightCam_", i, helper), calibrationHandler);
        dynCalib->syncInput.send(frameGroup);
        commandInput->send(std::make_shared<dai::DynamicCalibrationControl>(dai::DynamicCalibrationControl::Commands::LoadImage{}));
        auto coverage = coverageOutput->get<dai::CoverageData>();
        REQUIRE(coverage->coverageAcquired > 0.0f);
    }

    commandInput->send(std::make_shared<dai::DynamicCalibrationControl>(dai::DynamicCalibrationControl::Commands::Calibrate{true, true}));
    auto result = calibrationOutput->get<dai::DynamicCalibrationResult>();

    REQUIRE(result != nullptr);
    REQUIRE_FALSE(result->calibrationData.has_value());
    REQUIRE(result->info == "A multisensor pairwise recalibration changed the translation direction by 15 degrees or more");

    p.stop();
    p.wait();
}

TEST_CASE("DynamicCalibration: Recalibration on synthetic data with housing base.") {
    TestHelper helper;

    auto device = std::make_shared<dai::Device>();
    std::shared_ptr<dai::node::DynamicCalibration> dynCalib;
    auto p = makePipeline(device, dynCalib, false);

    auto coverageOutput = dynCalib->coverageOutput.createOutputQueue();
    auto commandInput = dynCalib->inputControl.createInputQueue();
    auto calibrationOutput = dynCalib->calibrationOutput.createOutputQueue();

    auto calibrationHandler = getHandler(true);
    device->setCalibration(calibrationHandler);

    auto leftToHousingBefore = calibrationHandler.getHousingCalibration(dai::CameraBoardSocket::CAM_C, dai::HousingCoordinateSystem::AUTO, false);
    auto rightToHousingBefore = calibrationHandler.getHousingCalibration(dai::CameraBoardSocket::CAM_B, dai::HousingCoordinateSystem::AUTO, false);

    auto group = stereoImageToMessageGroup(makeFilename("data/LeftCam_", 0, helper), makeFilename("data/RightCam_", 0, helper), calibrationHandler);
    p.start();
    dynCalib->syncInput.send(group);
    std::this_thread::sleep_for(0.5s);

    for(int i = 0; i < 7; i++) {
        auto frameGroup = stereoImageToMessageGroup(makeFilename("data/LeftCam_", i, helper), makeFilename("data/RightCam_", i, helper), calibrationHandler);
        dynCalib->syncInput.send(frameGroup);
        commandInput->send(std::make_shared<dai::DynamicCalibrationControl>(dai::DynamicCalibrationControl::Commands::LoadImage{}));
        auto coverage = coverageOutput->get<dai::CoverageData>();
        REQUIRE(coverage->coverageAcquired > 0.0f);
    }

    commandInput->send(std::make_shared<dai::DynamicCalibrationControl>(dai::DynamicCalibrationControl::Commands::Calibrate{true}));
    auto result = calibrationOutput->get<dai::DynamicCalibrationResult>();

    REQUIRE(result != nullptr);
    REQUIRE(result->calibrationData != std::nullopt);

    auto rotationMatrixOld = result->calibrationData->currentCalibration.getCameraRotationMatrix(dai::CameraBoardSocket::CAM_C, dai::CameraBoardSocket::CAM_B);
    std::vector<float> rvecOld = dai::matrix::rotationMatrixToVector(rotationMatrixOld);
    auto rotationMatrix = result->calibrationData->newCalibration.getCameraRotationMatrix(dai::CameraBoardSocket::CAM_C, dai::CameraBoardSocket::CAM_B);
    REQUIRE(std::fabs(rvecOld[0] - 0.01) < 0.00001);
    REQUIRE(std::fabs(rvecOld[1] - 0.01) < 0.00001);
    REQUIRE(std::fabs(rvecOld[2] - 0.01) < 0.00001);

    std::vector<float> rvec = dai::matrix::rotationMatrixToVector(rotationMatrix);
    INFO("Synthetic housing-base calibration uses RANSAC internally and is known to be non-deterministic.");
    CAPTURE(rvecOld[0], rvecOld[1], rvecOld[2], rvec[0], rvec[1], rvec[2]);
    float thresholdRotation = 3e-2f;
    REQUIRE(std::fabs(rvec[0]) < thresholdRotation);
    REQUIRE(std::fabs(rvec[1]) < thresholdRotation);
    REQUIRE(std::fabs(rvec[2]) < thresholdRotation);

    auto leftToHousing =
        result->calibrationData->newCalibration.getHousingCalibration(dai::CameraBoardSocket::CAM_C, dai::HousingCoordinateSystem::AUTO, false);
    auto rightToHousing =
        result->calibrationData->newCalibration.getHousingCalibration(dai::CameraBoardSocket::CAM_B, dai::HousingCoordinateSystem::AUTO, false);

    float thresholdTransform = 1e-5f;
    for(int row = 0; row < 4; ++row) {
        REQUIRE(std::fabs(leftToHousing[row][3] - leftToHousingBefore[row][3]) < thresholdTransform);
        REQUIRE(std::fabs(rightToHousing[row][3] - rightToHousingBefore[row][3]) < thresholdTransform);
    }

    p.stop();
    p.wait();
}

TEST_CASE("DynamicCalibration: Get metrics with housing base.") {
    TestHelper helper;

    auto device = std::make_shared<dai::Device>();
    std::shared_ptr<dai::node::DynamicCalibration> dynCalib;
    auto p = makePipeline(device, dynCalib, false);

    auto calibration = getHandler(true);
    auto coverageOutput = dynCalib->coverageOutput.createOutputQueue();
    auto commandInput = dynCalib->inputControl.createInputQueue();
    auto metricsOutput = dynCalib->metricsOutput.createOutputQueue();

    device->setCalibration(calibration);
    auto group = stereoImageToMessageGroup(makeFilename("data/LeftCam_", 0, helper), makeFilename("data/RightCam_", 0, helper), calibration);
    p.start();
    dynCalib->syncInput.send(group);
    std::this_thread::sleep_for(0.5s);

    for(int i = 0; i < 7; i++) {
        auto frameGroup = stereoImageToMessageGroup(makeFilename("data/LeftCam_", i, helper), makeFilename("data/RightCam_", i, helper), calibration);
        dynCalib->syncInput.send(frameGroup);
        commandInput->send(std::make_shared<dai::DynamicCalibrationControl>(dai::DynamicCalibrationControl::Commands::LoadImage{}));
        (void)coverageOutput->get<dai::CoverageData>();
    }

    commandInput->send(std::make_shared<dai::DynamicCalibrationControl>(dai::DynamicCalibrationControl::Commands::ComputeCalibrationMetrics{calibration}));

    bool failed = true;
    std::shared_ptr<dai::CalibrationMetrics> metrics;
    auto deadline = std::chrono::steady_clock::now() + 5s;
    while(std::chrono::steady_clock::now() < deadline) {
        metrics = metricsOutput->get<dai::CalibrationMetrics>(250ms, failed);
        if(!failed) break;
    }

    INFO("Waited up to 5 seconds for CalibrationMetrics");
    REQUIRE(!failed);
    REQUIRE(metrics != nullptr);

    p.stop();
    p.wait();
}

TEST_CASE("DynamicCalibration: Get metrics.") {
    TestHelper helper;

    auto device = std::make_shared<dai::Device>();
    std::shared_ptr<dai::node::DynamicCalibration> dynCalib;
    auto p = makePipeline(device, dynCalib, false);

    auto calibration = getHandler();
    auto coverageOutput = dynCalib->coverageOutput.createOutputQueue();
    auto commandInput = dynCalib->inputControl.createInputQueue();
    auto metricsOutput = dynCalib->metricsOutput.createOutputQueue();

    device->setCalibration(calibration);
    auto group = stereoImageToMessageGroup(makeFilename("data/LeftCam_", 0, helper), makeFilename("data/RightCam_", 0, helper), calibration);
    p.start();
    dynCalib->syncInput.send(group);
    std::this_thread::sleep_for(0.5s);

    // load image
    for(int i = 0; i < 7; i++) {
        auto group = stereoImageToMessageGroup(makeFilename("data/LeftCam_", i, helper), makeFilename("data/RightCam_", i, helper), calibration);
        dynCalib->syncInput.send(group);
        commandInput->send(std::make_shared<dai::DynamicCalibrationControl>(dai::DynamicCalibrationControl::Commands::LoadImage{}));
        auto coverage = coverageOutput->get<dai::CoverageData>();
    }

    commandInput->send(std::make_shared<dai::DynamicCalibrationControl>(dai::DynamicCalibrationControl::Commands::ComputeCalibrationMetrics{calibration}));

    bool failed = true;
    std::shared_ptr<dai::CalibrationMetrics> metrics;
    auto deadline = std::chrono::steady_clock::now() + 5s;
    while(std::chrono::steady_clock::now() < deadline) {
        metrics = metricsOutput->get<dai::CalibrationMetrics>(250ms, failed);
        if(!failed) break;
    }

    INFO("Waited up to 5 seconds for CalibrationMetrics");
    REQUIRE(!failed);
    REQUIRE(metrics != nullptr);

    p.stop();
    p.wait();
}
