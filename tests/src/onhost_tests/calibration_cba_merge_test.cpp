#include <catch2/catch_all.hpp>
#include <catch2/catch_test_macros.hpp>
#include <iostream>
#include <optional>
#include <unordered_map>

#include "depthai/common/CameraBoardSocket.hpp"
#include "depthai/depthai.hpp"
#include "depthai/device/EepromError.hpp"

class CalibrationData {
   public:
    explicit CalibrationData(std::shared_ptr<dai::Device> device) : device(std::move(device)) {}

    void capture() {
        try {
            mainBoardCalibrationData = device->readCalibration2();
        } catch(const dai::EepromError&) {
            mainBoardCalibrationData = std::nullopt;
        }
        mainBoardCalibrationDataCaptured = true;
    }

    void captureCBA(dai::CameraBoardSocket cameraSocket) {
        try {
            cbaCalibrationData[cameraSocket] = device->readCBACalibration2(cameraSocket);
        } catch(const dai::EepromError&) {
            cbaCalibrationData[cameraSocket] = std::nullopt;
        }
    }

    void restore() {
        if(mainBoardCalibrationDataCaptured) {
            if(mainBoardCalibrationData.has_value()) {
                device->flashCalibration(mainBoardCalibrationData.value());
            } else {
                device->flashEepromClear();
            }
        }

        for(const auto& [cameraSocket, data] : cbaCalibrationData) {
            restoreCBASocket(cameraSocket, data);
        }
        restored = true;
    }

    ~CalibrationData() {
        if(restored) return;

        if(mainBoardCalibrationDataCaptured) {
            try {
                if(mainBoardCalibrationData.has_value()) {
                    device->flashCalibration(mainBoardCalibrationData.value());
                } else {
                    device->flashEepromClear();
                }
            } catch(const std::exception& e) {
                std::cerr << "[EEPROM RESTORE FAILED] socket " << static_cast<int>(dai::CameraBoardSocket::AUTO) << ": " << e.what() << std::endl;
            }
        }

        for(const auto& [cameraSocket, data] : cbaCalibrationData) {
            try {
                restoreCBASocket(cameraSocket, data);
            } catch(const std::exception& e) {
                std::cerr << "[EEPROM RESTORE FAILED] socket " << static_cast<int>(cameraSocket) << ": " << e.what() << std::endl;
            }
        }
    }

   private:
    void restoreCBASocket(dai::CameraBoardSocket cameraSocket, const std::optional<dai::CBACalibrationHandler>& data) {
        if(data.has_value()) {
            device->flashCBACalibration(data.value(), cameraSocket);
        } else {
            device->flashCBAEepromClear(cameraSocket);
        }
    }

    std::shared_ptr<dai::Device> device;
    std::optional<dai::CalibrationHandler> mainBoardCalibrationData;
    bool mainBoardCalibrationDataCaptured = false;
    std::unordered_map<dai::CameraBoardSocket, std::optional<dai::CBACalibrationHandler>> cbaCalibrationData;
    bool restored = false;
};

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

static nlohmann::json loadCamDataCalibrationJson() {
    nlohmann::json cameraData = {{"cameraType", 0},
                                 {"distortionCoeff", nlohmann::json::array()},
                                 {"extrinsics",
                                  {{"rotationMatrix",
                                    {{0.9999265074729919, 0.006867990363389254, -0.009992731735110283},
                                     {-0.006895299535244703, 0.9999725818634033, -0.00270105991512537},
                                     {0.00997390691190958, 0.002769764279946685, 0.9999464154243469}}},
                                   {"specTranslation", {{"x", 6.548969268798828}, {"y", 0.01233222708106041}, {"z", 0.2119242250919342}}},
                                   {"toCameraSocket", static_cast<int>(dai::CameraBoardSocket::AUTO)},
                                   {"translation", {{"x", 6.548969268798828}, {"y", 0.01233222708106041}, {"z", 0.2119242250919342}}}}},
                                 {"height", 1080},
                                 {"intrinsicMatrix", {{613.353271484375, 0.0, 863.0769653320312}, {0.0, 612.0604248046875, 535.8486328125}, {0.0, 0.0, 1.0}}},
                                 {"lensPosition", 0},
                                 {"specHfovDeg", 0.0},
                                 {"width", 1920}};

    return {{"cameraData", {{static_cast<int>(dai::CameraBoardSocket::CBA), cameraData}}}};
}

static bool compareCameraData(const dai::CameraInfo& first, const dai::CameraInfo& second) {
    return (first.width == second.width) && (first.height == second.height) && (first.lensPosition == second.lensPosition)
           && (first.intrinsicMatrix == second.intrinsicMatrix) && (first.distortionCoeff == second.distortionCoeff)
           && (first.extrinsics.isEqualExtrinsics(second.extrinsics)) && (first.specHfovDeg == second.specHfovDeg) && (first.cameraType == second.cameraType);
}

TEST_CASE("CBA calibrations are merged with the main board's calibration") {
    std::shared_ptr<dai::Device> device = std::make_shared<dai::Device>();
    dai::Platform platform = device->getPlatform();

    // Look for connected CBAs
    std::vector<dai::CameraBoardSocket> cbaSockets;
    for(const auto& cameraSocket : device->getConnectedCameras()) {
        if(device->isCBAEepromAvailable(cameraSocket)) {
            cbaSockets.emplace_back(cameraSocket);
        }
    }
    if(cbaSockets.empty()) {
        return;
    }

    // Go over the compatible CBAs, store current flashed calibrations, followed by flashing with dummy data
    CalibrationData currentEepromData(device);
    for(const auto& cbaSocket : cbaSockets) {
        currentEepromData.captureCBA(cbaSocket);

        device->flashCBAEepromClear(cbaSocket);
        device->flashCBACalibration(dai::CBACalibrationHandler::fromJson(loadCamDataCalibrationJson()), cbaSocket);
    }

    // Apply the same for the main board
    REQUIRE(device->isEepromAvailable());
    currentEepromData.capture();

    device->flashEepromClear();
    device->flashCalibration(loadGeneralCalibration());

    // Check if the merged runtime calibration contains the camera data from the CBAs before pipeline starts
    dai::CalibrationHandler mergedCalibrationBeforeStart = device->getCalibration();
    for(const auto& cbaSocket : cbaSockets) {
        REQUIRE(mergedCalibrationBeforeStart.hasCameraCalibration(cbaSocket));

        const auto mainCameraData = mergedCalibrationBeforeStart.getEepromData().cameraData.at(cbaSocket);
        const auto cbaCameraData = device->readCBACalibration2(cbaSocket).getEepromData().cameraData.at(dai::CameraBoardSocket::CBA);
        REQUIRE(compareCameraData(mainCameraData, cbaCameraData));
    }

    // Starting the pipeline should keep the already prepared runtime calibration intact
    dai::Pipeline pipeline(device);
    auto logger = pipeline.create<dai::node::SystemLogger>();
    auto outputQueue = logger->out.createOutputQueue(1, false);
    pipeline.start();
    dai::CalibrationHandler mergedCalibration = device->getCalibration();
    for(const auto& cbaSocket : cbaSockets) {
        REQUIRE(mergedCalibration.hasCameraCalibration(cbaSocket));

        const auto mainCameraData = mergedCalibration.getEepromData().cameraData.at(cbaSocket);
        const auto cbaCameraData = device->readCBACalibration2(cbaSocket).getEepromData().cameraData.at(dai::CameraBoardSocket::CBA);
        REQUIRE(compareCameraData(mainCameraData, cbaCameraData));
    }

    REQUIRE_NOTHROW(currentEepromData.restore());
}

TEST_CASE("Calibration's cameraData present on the main board has priority and is not replaced with camera socket data present on the CBAs") {
    std::shared_ptr<dai::Device> device = std::make_shared<dai::Device>();
    dai::Platform platform = device->getPlatform();

    // Look for connected CBAs
    std::vector<dai::CameraBoardSocket> cbaSockets;
    for(const auto& cameraSocket : device->getConnectedCameras()) {
        if(device->isCBAEepromAvailable(cameraSocket)) {
            cbaSockets.emplace_back(cameraSocket);
        }
    }
    if(cbaSockets.empty()) {
        return;
    }

    // Go over the compatible CBAs, store current flashed calibrations, followed by flashing with dummy data
    CalibrationData currentEepromData(device);
    for(const auto& cbaSocket : cbaSockets) {
        currentEepromData.captureCBA(cbaSocket);

        device->flashCBAEepromClear(cbaSocket);
        device->flashCBACalibration(dai::CBACalibrationHandler::fromJson(loadCamDataCalibrationJson()), cbaSocket);
    }

    // Apply the same for the main board
    REQUIRE(device->isEepromAvailable());
    currentEepromData.capture();

    device->flashEepromClear();
    device->flashCalibration(loadGeneralCalibrationDefaultCameraData());

    // Check that the merged runtime calibration keeps the main board's camera data before pipeline start (since the main board should have the priority in
    // case of duplicates)
    dai::CalibrationHandler mergedCalibrationBeforeStart = device->getCalibration();
    for(const auto& cbaSocket : cbaSockets) {
        REQUIRE(mergedCalibrationBeforeStart.hasCameraCalibration(cbaSocket));

        const auto mainCameraData = mergedCalibrationBeforeStart.getEepromData().cameraData.at(cbaSocket);
        const auto cbaCameraData = device->readCBACalibration2(cbaSocket).getEepromData().cameraData.at(dai::CameraBoardSocket::CBA);
        REQUIRE(!compareCameraData(mainCameraData, cbaCameraData));
    }

    // Starting the pipeline should keep the already prepared runtime calibration intact
    dai::Pipeline pipeline(device);
    auto logger = pipeline.create<dai::node::SystemLogger>();
    auto outputQueue = logger->out.createOutputQueue(1, false);
    pipeline.start();
    dai::CalibrationHandler mergedCalibration = device->getCalibration();
    for(const auto& cbaSocket : cbaSockets) {
        REQUIRE(mergedCalibration.hasCameraCalibration(cbaSocket));

        const auto mainCameraData = mergedCalibration.getEepromData().cameraData.at(cbaSocket);
        const auto cbaCameraData = device->readCBACalibration2(cbaSocket).getEepromData().cameraData.at(dai::CameraBoardSocket::CBA);
        REQUIRE(!compareCameraData(mainCameraData, cbaCameraData));
    }

    REQUIRE_NOTHROW(currentEepromData.restore());
}
