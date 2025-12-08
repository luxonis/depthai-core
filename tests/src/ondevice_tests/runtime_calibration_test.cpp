#include <catch2/catch_all.hpp>

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
