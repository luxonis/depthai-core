#include <algorithm>
#include <catch2/catch_all.hpp>
#include <catch2/catch_test_macros.hpp>
#include <depthai/depthai.hpp>

#include "depthai/pipeline/datatype/ImgFrame.hpp"
#include "depthai/pipeline/datatype/MessageGroup.hpp"
#include "depthai/pipeline/node/Camera.hpp"
#include "depthai/pipeline/node/Sync.hpp"

// -----------------------------------------------------------------------------
// Undistortion is only applied on new EEPROM formats
// Purpose:
//   Ensures that requesting camera output with enableUndistortion=true only
//   changes the image when the EEPROM format supports distortion correction.
// -----------------------------------------------------------------------------
TEST_CASE("Undistorted output changes only on new EEPROM") {
    dai::Pipeline pipeline;

    // Read the device's current calibration and force old EEPROM format
    auto device = pipeline.getDefaultDevice();
    auto calib = device->readCalibration();
    auto eeprom = calib.getEepromData();
    bool newEEPROM = false;

    auto socket = dai::CameraBoardSocket::CAM_B;

    // Force <= 85 degrees FOV
    eeprom.cameraData[socket].intrinsicMatrix[0][0] = 1000000.0f;  // huge focal length <=> small fov
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
    auto sync = pipeline.create<dai::node::Sync>();

    auto* distortedOutput = camera->requestOutput({640, 400}, dai::ImgFrame::Type::NV12, dai::ImgResizeMode::CROP);
    auto* undistortedOutput = camera->requestOutput({640, 400}, dai::ImgFrame::Type::NV12, dai::ImgResizeMode::CROP, std::nullopt, true);
    REQUIRE(distortedOutput != nullptr);
    REQUIRE(undistortedOutput != nullptr);

    distortedOutput->link(sync->inputs["distorted"]);
    undistortedOutput->link(sync->inputs["undistorted"]);

    auto queue = sync->out.createOutputQueue();

    pipeline.start();

    bool foundDifferentFramePair = false;
    for(int i = 0; i < 5; i++) {
        auto frames = queue->get<dai::MessageGroup>();
        REQUIRE(frames != nullptr);

        auto distortedFrame = frames->get<dai::ImgFrame>("distorted");
        auto undistortedFrame = frames->get<dai::ImgFrame>("undistorted");
        REQUIRE(distortedFrame != nullptr);
        REQUIRE(undistortedFrame != nullptr);

        REQUIRE(distortedFrame->getWidth() == undistortedFrame->getWidth());
        REQUIRE(distortedFrame->getHeight() == undistortedFrame->getHeight());
        REQUIRE(distortedFrame->getType() == undistortedFrame->getType());

        const auto distortedData = distortedFrame->getData();
        const auto undistortedData = undistortedFrame->getData();
        const bool sameData = distortedData.size() == undistortedData.size() && std::equal(distortedData.begin(), distortedData.end(), undistortedData.begin());
        if(!sameData) {
            foundDifferentFramePair = true;
            break;
        }
    }

    pipeline.stop();
    if(newEEPROM) {
        REQUIRE(foundDifferentFramePair);
    } else {
        REQUIRE_FALSE(foundDifferentFramePair);
    }
}
