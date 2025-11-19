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
#include "depthai/common/CameraBoardSocket.hpp"


class EepromRestoreGuard {
public:
    explicit EepromRestoreGuard(std::shared_ptr<dai::Device> dev)
        : device(std::move(dev)), factoryCalib(device->readCalibration()) {}

    ~EepromRestoreGuard() {
        try {
            device->flashCalibration(factoryCalib);
        } catch(const std::exception& e) {
            std::cerr << "[EEPROM RESTORE FAILED] " << e.what() << std::endl;
        }
    }

private:
    std::shared_ptr<dai::Device> device;
    dai::CalibrationHandler factoryCalib;
};



TEST_CASE("EEPROM modify + round-trip") {
    auto device = std::make_shared<dai::Device>();
    REQUIRE(device != nullptr);

    // Always restore
    EepromRestoreGuard restore(device);

    //
    // 1. Read the factory EEPROM
    //
    dai::CalibrationHandler factoryCalib = device->readCalibration();
    dai::EepromData eeprom = factoryCalib.getEepromData();

    //
    // 2. Modify ONLY allowed fields
    //
    eeprom.housingExtrinsics.toCameraSocket = dai::CameraBoardSocket::CAM_B;
    eeprom.housingExtrinsics.translation = {1.f, 2.f, 3.f};
    eeprom.housingExtrinsics.specTranslation = {1.f, 2.f, 3.f};

    eeprom.housingExtrinsics.rotationMatrix = {
        {1.f, 1.f, 0.f},
        {0.f, 1.f, 0.f},
        {0.f, 0.f, 1.f}
    };

    //
    // 3. Repack
    //
    dai::CalibrationHandler modifiedCalib(eeprom);

    //
    // 4. Flash — ALLOWED because we didn't touch protected fields
    //
    REQUIRE_NOTHROW(device->flashCalibration(modifiedCalib));

    //
    // 5. Read back and test
    //
    auto readBack = device->readCalibration();
    auto eeprom2 = readBack.getEepromData();

    REQUIRE(eeprom2.housingExtrinsics.translation.x == Catch::Approx(1.f));
    REQUIRE(eeprom2.housingExtrinsics.translation.y == Catch::Approx(2.f));
    REQUIRE(eeprom2.housingExtrinsics.translation.z == Catch::Approx(3.f));

    REQUIRE(eeprom2.housingExtrinsics.toCameraSocket == dai::CameraBoardSocket::CAM_B);

    std::vector<std::vector<float>> expected = {
        {1.f, 1.f, 0.f},
        {0.f, 1.f, 0.f},
        {0.f, 0.f, 1.f}
    };

    for(int r = 0; r < 3; r++)
        for(int c = 0; c < 3; c++)
            REQUIRE(eeprom2.housingExtrinsics.rotationMatrix[r][c] == Catch::Approx(expected[r][c]));
}

