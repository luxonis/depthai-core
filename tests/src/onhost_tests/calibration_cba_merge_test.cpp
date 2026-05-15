#include <catch2/catch_all.hpp>
#include <catch2/catch_test_macros.hpp>

#include "depthai/common/CameraBoardSocket.hpp"
#include "depthai/depthai.hpp"

static dai::CalibrationHandler loadGeneralCalibration() {
    dai::EepromData eepromData;
    eepromData.productName = "CBA merge test";
    eepromData.boardName = "CBA merge test";
    return dai::CalibrationHandler(eepromData);
}

static dai::CalibrationHandler loadGeneralCalibrationDefaultCameraData() {
    dai::EepromData eepromData;
    eepromData.productName = "CBA merge test";
    eepromData.boardName = "CBA merge test";

    for(const auto cameraSocket : {dai::CameraBoardSocket::CAM_A,
                                   dai::CameraBoardSocket::CAM_B,
                                   dai::CameraBoardSocket::CAM_C,
                                   dai::CameraBoardSocket::CAM_D,
                                   dai::CameraBoardSocket::CAM_E,
                                   dai::CameraBoardSocket::CAM_F,
                                   dai::CameraBoardSocket::CAM_G,
                                   dai::CameraBoardSocket::CAM_H}) {
        dai::CameraInfo cameraInfo;
        cameraInfo.intrinsicMatrix = {{1.0f, 0.0f, 0.0f}, {0.0f, 1.0f, 0.0f}, {0.0f, 0.0f, 1.0f}};
        eepromData.cameraData.emplace(cameraSocket, cameraInfo);
    }

    return dai::CalibrationHandler(eepromData);
}

static nlohmann::json loadCamDataCalibrationJson(dai::CameraBoardSocket cameraSocket) {
    nlohmann::json cameraData = {{"cameraType", 0},
                                 {"distortionCoeff", nlohmann::json::array()},
                                 {"extrinsics",
                                  {{"rotationMatrix", {{1.0, 0.0, 0.0}, {0.0, 1.0, 0.0}, {0.0, 0.0, 1.0}}},
                                   {"specTranslation", {{"x", 0.0}, {"y", 0.0}, {"z", 0.0}}},
                                   {"toCameraSocket", static_cast<int>(dai::CameraBoardSocket::AUTO)},
                                   {"translation", {{"x", 0.0}, {"y", 0.0}, {"z", 0.0}}}}},
                                 {"height", 1080},
                                 {"intrinsicMatrix", {{613.353271484375, 0.0, 863.0769653320312}, {0.0, 612.0604248046875, 535.8486328125}, {0.0, 0.0, 1.0}}},
                                 {"lensPosition", 0},
                                 {"specHfovDeg", 0.0},
                                 {"width", 1920}};

    return {{"cameraData", {{static_cast<int>(cameraSocket), cameraData}}}};
}

static bool compareCameraData(const dai::CameraInfo& first, const dai::CameraInfo& second) {
    return (first.width == second.width) && (first.height == second.height) && (first.lensPosition == second.lensPosition)
           && (first.intrinsicMatrix == second.intrinsicMatrix) && (first.distortionCoeff == second.distortionCoeff)
           && (first.extrinsics.isEqualExtrinsics(second.extrinsics)) && (first.specHfovDeg == second.specHfovDeg) && (first.cameraType == second.cameraType);
}

TEST_CASE("CBA calibrations are merged with the main board's calibration") {
    std::shared_ptr<dai::Device> device = std::make_shared<dai::Device>();
    dai::Platform platform = device->getPlatform();
    // Currently available only on RVC2
    if(platform == dai::Platform::RVC4) {
        return;
    }

    // Look for connected CBAs
    std::vector<dai::CameraBoardSocket> cbaSockets;
    for(const auto& cameraSocket : device->getConnectedCameras()) {
        if(device->isEepromAvailable(cameraSocket)) {
            cbaSockets.emplace_back(cameraSocket);
        }
    }
    if(cbaSockets.empty()) {
        return;
    }

    // TO DO: Should this test change factory calibration - even though it only does this in case it's not present. It tests the factory calibration reading
    // tho. Go over the compatible CBAs, check for their factory calibration contents, flash with dummy data if missing
    for(const auto& cbaSocket : cbaSockets) {
        if(!device->readFactoryCalibrationOrDefault(cbaSocket).hasCameraCalibration(cbaSocket)) {
            // Missing factory calibration, flash factory calibration with dummy data
            device->flashFactoryEepromClear(cbaSocket);
            device->flashFactoryCalibration(dai::CalibrationHandler::fromJson(loadCamDataCalibrationJson(cbaSocket)), cbaSocket);
        }
        // Flash the user calibration with the factory data
        device->flashEepromClear(cbaSocket);
        device->factoryResetCalibration(cbaSocket);
    }

    // Apply the same for the main board
    REQUIRE(device->isEepromAvailable());
    if(!device->readFactoryCalibrationOrDefault(dai::CameraBoardSocket::AUTO).hasCalibrationData()) {
        // Missing factory calibration, flash general calibration data without synthetic camera entries
        device->flashFactoryEepromClear();
        device->flashFactoryCalibration(loadGeneralCalibration());
    }
    // Flash the user calibration with the factory data
    device->flashEepromClear();
    device->factoryResetCalibration();

    // Check if the merged calibrations contain the camera data from the CBAs
    dai::CalibrationHandler mergedCalibration = device->readCalibration2();
    dai::CalibrationHandler mergedFactoryCalibration = device->readFactoryCalibration();

    for(const auto& cbaSocket : cbaSockets) {
        REQUIRE(mergedCalibration.hasCameraCalibration(cbaSocket));
        REQUIRE(mergedFactoryCalibration.hasCameraCalibration(cbaSocket));

        const auto mainCameraData = mergedCalibration.getEepromData().cameraData.at(cbaSocket);
        const auto cbaCameraData = device->readCalibration2(cbaSocket).getEepromData().cameraData.at(cbaSocket);
        REQUIRE(compareCameraData(mainCameraData, cbaCameraData) == true);

        const auto mainFactoryCameraData = mergedFactoryCalibration.getEepromData().cameraData.at(cbaSocket);
        const auto cbaFactoryCameraData = device->readFactoryCalibration(cbaSocket).getEepromData().cameraData.at(cbaSocket);
        REQUIRE(compareCameraData(mainFactoryCameraData, cbaFactoryCameraData) == true);
    }
}

TEST_CASE("Calibration's cameraData present on the main board has priority and is not replaced with camera socket data present on the CBAs") {
    std::shared_ptr<dai::Device> device = std::make_shared<dai::Device>();
    dai::Platform platform = device->getPlatform();
    // Currently available only on RVC2
    if(platform == dai::Platform::RVC4) {
        return;
    }

    // Look for connected CBAs
    std::vector<dai::CameraBoardSocket> cbaSockets;
    for(const auto& cameraSocket : device->getConnectedCameras()) {
        if(device->isEepromAvailable(cameraSocket)) {
            cbaSockets.emplace_back(cameraSocket);
        }
    }
    if(cbaSockets.empty()) {
        return;
    }

    // Go over the compatible CBAs, flash with dummy data
    for(const auto& cbaSocket : cbaSockets) {
        device->flashEepromClear(cbaSocket);
        device->flashCalibration(dai::CalibrationHandler::fromJson(loadCamDataCalibrationJson(cbaSocket)), cbaSocket);
    }

    // Apply the same for the main board
    REQUIRE(device->isEepromAvailable());
    device->flashEepromClear();
    device->flashCalibration(loadGeneralCalibrationDefaultCameraData());

    // Check that the merged calibration doesn't contain the camera data from the CBAs (since the main board should have the priority)
    dai::CalibrationHandler mergedCalibration = device->readCalibration2();
    for(const auto& cbaSocket : cbaSockets) {
        REQUIRE(mergedCalibration.hasCameraCalibration(cbaSocket));

        const auto mainCameraData = mergedCalibration.getEepromData().cameraData.at(cbaSocket);
        const auto cbaCameraData = device->readCalibration2(cbaSocket).getEepromData().cameraData.at(cbaSocket);
        REQUIRE(compareCameraData(mainCameraData, cbaCameraData) == false);
    }
}
