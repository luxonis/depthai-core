#include <algorithm>
#include <catch2/catch_all.hpp>
#include <catch2/catch_test_macros.hpp>
#include <depthai/depthai.hpp>

#include "depthai/pipeline/datatype/ImgFrame.hpp"
#include "depthai/pipeline/node/Camera.hpp"

// -----------------------------------------------------------------------------
// Undistortion disabled on old EEPROM (stereoEnableDistortionCorrection=false)
// Purpose:
//   Ensures that when a device has an older EEPROM format
//   (stereoEnableDistortionCorrection == false), requesting camera output with
//   enableUndistortion=true does NOT produce distortion coefficients in the
//   undistortion request when the EEPROM format is old.
// -----------------------------------------------------------------------------
TEST_CASE("Undistortion disabled on old EEPROM") {
    dai::Pipeline pipeline;

    // Read the device's current calibration and force old EEPROM format
    auto device = pipeline.getDefaultDevice();
    auto calib = device->readCalibration();
    auto eeprom = calib.getEepromData();
    bool newEEPROM = false;

    auto socket = dai::CameraBoardSocket::CAM_B;

    // Force <= 85 degrees FOV
    eeprom.cameraData[socket].intrinsicMatrix[0][0] = 1000000.0f; // huge focal length <=> small fov
    eeprom.cameraData[socket].width = 640;
    eeprom.cameraData[socket].distortionCoeff = {3.0f, 1.0f, 4.0f, 1.0f, 5.0f, 9.0f, 2.0f, 6.0f, 5.0f, 3.0f, 5.0f, 8.0f, 9.0f, 7.0f};

    SECTION("stereoEnableDistortionCorrection = false (old EEPROM format)") {
        newEEPROM = false;
    }

    SECTION("stereoEnableDistortionCorrection = true (new EEPROM format)") {
        newEEPROM = true;
    }

    eeprom.stereoEnableDistortionCorrection = newEEPROM;
    device->setCalibration(dai::CalibrationHandler(eeprom));

    auto camera = pipeline.create<dai::node::Camera>()->build(socket);

    // Request output with enableUndistortion explicitly set to true
    auto* output = camera->requestOutput({640, 400}, dai::ImgFrame::Type::NV12, dai::ImgResizeMode::CROP, std::nullopt, true);
    REQUIRE(output != nullptr);
    auto queue = output->createOutputQueue();

    pipeline.start();
    auto frame = queue->get<dai::ImgFrame>();
    REQUIRE(frame != nullptr);
    pipeline.stop();

    auto distCoeffs = frame->transformation.getDistortionCoefficients();

    if(newEEPROM) {
        // New EEPROM: firmware applies undistortion and then zeros out the coefficients
        bool allZero = distCoeffs.empty() || std::all_of(distCoeffs.begin(), distCoeffs.end(), [](float v) { return v == 0.0f; });
        REQUIRE(allZero);
    } else {
        // Old EEPROM: firmware skips undistortion, distortion coefficients are passed through
        bool someNonZero = !distCoeffs.empty() && std::any_of(distCoeffs.begin(), distCoeffs.end(), [](float v) { return v != 0.0f; });
        REQUIRE(someNonZero);
    }
}
