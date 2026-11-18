#include <catch2/catch_test_macros.hpp>

#include "depthai/device/Device.hpp"
#include "depthai/device/HealthCheck.hpp"

TEST_CASE("Camera functionality health check does not require a full-FOV sensor mode") {
    const auto [found, deviceInfo] = dai::Device::getAnyAvailableDevice();
    REQUIRE(found);

    const dai::HealthCheckConfig config(false, false, true, false, false, false, false);
    const auto metrics = dai::Device::performHealthCheck(deviceInfo, config);

    INFO(metrics.toString());
    CHECK(metrics.cameraFunctionality == dai::HealthCheckResult::PASS);
    CHECK(metrics.powerSupplyFunctionality == dai::HealthCheckResult::NOT_RUN);
    CHECK(metrics.cameraCalibration == dai::HealthCheckResult::NOT_RUN);
}
