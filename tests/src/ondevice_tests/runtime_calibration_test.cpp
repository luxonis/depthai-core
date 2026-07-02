#include <catch2/catch_all.hpp>

#include <algorithm>

#include "depthai/common/CameraBoardSocket.hpp"
#include "depthai/common/CameraFeatures.hpp"
#include "depthai/common/CameraSensorType.hpp"
#include "depthai/common/StereoPair.hpp"
#include "depthai/depthai.hpp"
#include "depthai/utility/CompilerWarnings.hpp"

TEST_CASE("Test runtime calibration") {
    using namespace std;
    using namespace std::chrono;
    using namespace std::chrono_literals;

    float epsilon = std::numeric_limits<float>::epsilon();

    dai::Device device;

    dai::CalibrationHandler calibHandler, calibHandlerReadBack;

    dai::CameraBoardSocket cameraId = dai::CameraBoardSocket::CAM_B;
    std::vector<std::vector<float>> intrinsics = {{1000.0f, 0.0f, 320.0f}, {0.0f, 1000.0f, 240.0f}, {0.0f, 0.0f, 1.0f}};
    std::tuple<int, int> frameSize = {1280, 800};

    calibHandler.setCameraIntrinsics(cameraId, intrinsics, frameSize);
    device.setCalibration(calibHandler);

    try {
        calibHandlerReadBack = device.getCalibration();
        auto readBackIntrinsics = calibHandlerReadBack.getCameraIntrinsics(cameraId);

        for(size_t i = 0; i < intrinsics.size(); ++i) {
            for(size_t j = 0; j < intrinsics[i].size(); ++j) {
                REQUIRE(std::abs(intrinsics[i][j] - readBackIntrinsics[i][j]) < epsilon);
            }
        }

    } catch(const std::exception& e) {
        FAIL("Failed to read back intrinsics: " << e.what());
    }

    std::vector<float> distortionCoefficients;
    for(size_t i = 0; i < 14; ++i) {
        distortionCoefficients.push_back(i);
    }

    calibHandler.setDistortionCoefficients(cameraId, distortionCoefficients);
    device.setCalibration(calibHandler);

    try {
        calibHandlerReadBack = device.getCalibration();
        auto readBackDistortionCoefficients = calibHandlerReadBack.getDistortionCoefficients(cameraId);

        REQUIRE(readBackDistortionCoefficients.size() == distortionCoefficients.size());
        for(size_t i = 0; i < distortionCoefficients.size(); ++i) {
            REQUIRE(std::abs(distortionCoefficients[i] - readBackDistortionCoefficients[i]) < epsilon);
        }

    } catch(const std::exception& e) {
        FAIL("Failed to read back distortion coefficients: " << e.what());
    }
}

TEST_CASE("Test device setCalibration before pipeline build") {
    dai::Pipeline p;
    auto camQ = p.create<dai::node::Camera>()->build(dai::CameraBoardSocket::CAM_A)->requestOutput({640, 480})->createOutputQueue();
    dai::CalibrationHandler calibHandler = p.getDefaultDevice()->getCalibration();
    calibHandler.validateCalibrationHandler();
    calibHandler.setDeviceName("test_device_name");
    p.getDefaultDevice()->setCalibration(calibHandler);

    auto calibPreBuild = p.getDefaultDevice()->getCalibration();
    REQUIRE(calibPreBuild.getEepromData().deviceName == "test_device_name");

    p.build();

    auto calibPostBuild = p.getDefaultDevice()->getCalibration();
    REQUIRE(calibPostBuild.getEepromData().deviceName == "test_device_name");

    p.start();

    auto calibPostStart = p.getDefaultDevice()->getCalibration();
    REQUIRE(calibPostStart.getEepromData().deviceName == "test_device_name");

    p.stop();
}

TEST_CASE("Test pipeline setCalibration before pipeline build") {
    dai::Pipeline p;
    auto camQ = p.create<dai::node::Camera>()->build(dai::CameraBoardSocket::CAM_A)->requestOutput({640, 480})->createOutputQueue();
    dai::CalibrationHandler calibHandler = p.getDefaultDevice()->getCalibration();
    calibHandler.validateCalibrationHandler();
    calibHandler.setDeviceName("test_device_name");
    p.setCalibrationData(calibHandler);

    p.start();

    auto calibPostStart = p.getDefaultDevice()->getCalibration();
    REQUIRE(calibPostStart.getEepromData().deviceName == "test_device_name");

    p.stop();
}

TEST_CASE("getStereoPairs filters out pairs with z-dominant translation") {
    dai::Device device;

    const auto originalCalibration = device.getCalibration();
    const auto originalFeatures = device.getConnectedCameraFeatures();
    const auto originalPairs = device.getStereoPairs();

    if(originalPairs.empty()) {
        SKIP("Device does not expose any stereo pairs");
    }

    const auto pairUnderTest = originalPairs.front();
    auto mockedFeatures = originalFeatures;
    for(auto& feature : mockedFeatures) {
        if(feature.socket == pairUnderTest.left || feature.socket == pairUnderTest.right) {
            feature.sensorName = "OV9282";
            feature.width = 1280;
            feature.height = 800;
            feature.supportedTypes = {dai::CameraSensorType::MONO};
        }
    }

    auto calibration = originalCalibration;
    const std::vector<std::vector<float>> intrinsics = {{1000.0f, 0.0f, 640.0f}, {0.0f, 1000.0f, 400.0f}, {0.0f, 0.0f, 1.0f}};
    calibration.setCameraIntrinsics(pairUnderTest.left, intrinsics, 1280, 800);
    calibration.setCameraIntrinsics(pairUnderTest.right, intrinsics, 1280, 800);
    calibration.setCameraExtrinsics(pairUnderTest.left, pairUnderTest.right, {{1.0f, 0.0f, 0.0f}, {0.0f, 1.0f, 0.0f}, {0.0f, 0.0f, 1.0f}}, {1.0f, 1.0f, 5.0f}, {1.0f, 1.0f, 5.0f});

    device.overrideCameraFeatures(mockedFeatures);
    device.setCalibration(calibration);

    const auto filteredPairs = device.getStereoPairs();
    const bool pairStillPresent = std::any_of(filteredPairs.begin(), filteredPairs.end(), [&](const dai::StereoPair& candidate) {
        return (candidate.left == pairUnderTest.left && candidate.right == pairUnderTest.right)
               || (candidate.left == pairUnderTest.right && candidate.right == pairUnderTest.left);
    });

    device.setCalibration(originalCalibration);
    device.overrideCameraFeatures(originalFeatures);

    REQUIRE_FALSE(pairStillPresent);
}
