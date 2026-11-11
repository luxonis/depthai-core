#include <algorithm>
#include <catch2/catch_all.hpp>
#include <cstdlib>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "depthai/depthai.hpp"
#include "depthai/device/EepromError.hpp"

// Writes user and factory calibration. Run it only on a disposable board whose EEPROM is erased:
// the read-only checks at the start fail on any board that holds calibration, before anything is written.

namespace {

dai::CalibrationHandler calibrationWithFocal(float focal) {
    dai::CalibrationHandler calibration;
    calibration.setCameraIntrinsics(dai::CameraBoardSocket::CAM_A, {{focal, 0, 320}, {0, 810, 240}, {0, 0, 1}}, 640, 480);
    return calibration;
}

float focal(const dai::CalibrationHandler& calibration) {
    return calibration.getCameraIntrinsics(dai::CameraBoardSocket::CAM_A)[0][0];
}

bool erased(const std::vector<std::uint8_t>& bytes) {
    return !bytes.empty() && std::all_of(bytes.begin(), bytes.end(), [](std::uint8_t value) { return value == 0xff; });
}

struct FactoryPermissionGuard {
    static constexpr const char* NAME = "DEPTHAI_ALLOW_FACTORY_FLASHING";
    std::optional<std::string> previous;

    static int put(const char* value) {
#ifdef _WIN32
        return _putenv_s(NAME, value ? value : "");
#else
        return value ? setenv(NAME, value, 1) : unsetenv(NAME);
#endif
    }

    FactoryPermissionGuard() {
        if(const char* value = std::getenv(NAME)) previous = value;
        REQUIRE(put("") == 0);
    }

    ~FactoryPermissionGuard() {
        put(previous ? previous->c_str() : nullptr);
    }
};

}  // namespace

TEST_CASE("Board calibration persists across sessions and factory reset") {
    // Discovery info, not getDeviceInfo(): a connected device reports its booted state, which would not reconnect through Gate
    const auto [found, info] = dai::Device::getAnyAvailableDevice();
    REQUIRE(found);
    auto device = std::make_shared<dai::Device>(info);

    REQUIRE(device->isEepromAvailable());
    REQUIRE_FALSE(device->isCalibrationAvailable());
    REQUIRE(device->tryGetCalibration() == nullptr);
    REQUIRE_FALSE(device->readCalibration().hasCalibrationData());
    REQUIRE_THROWS_AS(device->readCalibration2(), dai::EepromError);
    REQUIRE_THROWS_AS(device->readFactoryCalibration(), dai::EepromError);
    REQUIRE(erased(device->readCalibrationRaw()));
    REQUIRE(erased(device->readFactoryCalibrationRaw()));
    REQUIRE_THROWS_AS(device->factoryResetCalibration(), dai::EepromError);

    {
        FactoryPermissionGuard noPermission;
        device->flashCalibration(calibrationWithFocal(800));
        REQUIRE(focal(device->readCalibration2()) == 800);
        auto changed = calibrationWithFocal(801).getEepromData();
        changed.boardName = "test-changed";
        REQUIRE_THROWS_WITH(device->flashCalibration(dai::CalibrationHandler(changed)), Catch::Matchers::ContainsSubstring("PROTECTED permissions"));
        REQUIRE_THROWS_WITH(device->flashFactoryCalibration(calibrationWithFocal(900)), Catch::Matchers::ContainsSubstring("not allowed"));
        REQUIRE_THROWS_WITH(device->flashEepromClear(), Catch::Matchers::ContainsSubstring("not allowed"));
        REQUIRE(focal(device->readCalibration2()) == 800);
    }

    REQUIRE(focal(device->getCalibration()) == 800);
    device->setCalibration(calibrationWithFocal(1000));
    REQUIRE(focal(device->getCalibration()) == 1000);
    REQUIRE(focal(device->readCalibration2()) == 800);
    device->close();
    device.reset();

    device = std::make_shared<dai::Device>(info);
    REQUIRE(focal(device->getCalibration()) == 800);
    device->flashFactoryCalibration(calibrationWithFocal(900));
    REQUIRE(focal(device->readFactoryCalibration()) == 900);
    REQUIRE(focal(device->getCalibration()) == 800);
    device->setCalibration(calibrationWithFocal(1000));
    device->flashCalibration(calibrationWithFocal(801));
    REQUIRE(focal(device->getCalibration()) == 801);
    REQUIRE(focal(device->readCalibration2()) == 801);
    device->setCalibration(std::nullopt);
    REQUIRE(focal(device->getCalibration()) == 801);

    const auto factoryBytes = device->readFactoryCalibrationRaw();
    device->factoryResetCalibration();
    REQUIRE(focal(device->readCalibration2()) == 900);
    REQUIRE(focal(device->getCalibration()) == 801);
    device->setCalibration(std::nullopt);
    REQUIRE(focal(device->getCalibration()) == 801);
    REQUIRE(device->readCalibrationRaw() == factoryBytes);
    device->close();
    device.reset();

    device = std::make_shared<dai::Device>(info);
    REQUIRE(focal(device->getCalibration()) == 900);
    device->flashEepromClear();
    REQUIRE_THROWS_AS(device->readCalibration2(), dai::EepromError);
    REQUIRE(focal(device->getCalibration()) == 900);
    REQUIRE(device->readFactoryCalibrationRaw() == factoryBytes);
    REQUIRE(focal(device->readFactoryCalibration()) == 900);
    device->flashFactoryEepromClear();
    device->flashFactoryEepromClear();
    REQUIRE(device->isEepromAvailable());
    REQUIRE_THROWS_AS(device->readFactoryCalibration(), dai::EepromError);
    device->close();
}
