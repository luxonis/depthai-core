#include <catch2/catch_all.hpp>
#include <depthai/depthai.hpp>
#include <memory>

TEST_CASE("DynamicCalibration - Commands", "[DynamicCalibrationControl]") {
    using DCC = dai::DynamicCalibrationControl;

    SECTION("Calibrate command") {
        auto cmd = DCC::calibrate(true);
        REQUIRE(std::holds_alternative<DCC::Commands::Calibrate>(cmd->command));
        auto& c = std::get<DCC::Commands::Calibrate>(cmd->command);
        REQUIRE(c.force == true);
    }

    SECTION("CalibrationQuality command") {
        auto cmd = DCC::calibrationQuality(false);
        REQUIRE(std::holds_alternative<DCC::Commands::CalibrationQuality>(cmd->command));
        auto& c = std::get<DCC::Commands::CalibrationQuality>(cmd->command);
        REQUIRE(c.force == false);
    }

    SECTION("StartCalibration command with custom periods") {
        auto cmd = DCC::startCalibration(1.0f, 10.0f);
        REQUIRE(std::holds_alternative<DCC::Commands::StartCalibration>(cmd->command));
        auto& c = std::get<DCC::Commands::StartCalibration>(cmd->command);
        REQUIRE(c.loadImagePeriod == Catch::Approx(1.0f));
        REQUIRE(c.calibrationPeriod == Catch::Approx(10.0f));
    }

    SECTION("StopCalibration command") {
        auto cmd = DCC::stopCalibration();
        REQUIRE(std::holds_alternative<DCC::Commands::StopCalibration>(cmd->command));
    }

    SECTION("LoadImage command") {
        auto cmd = DCC::loadImage();
        REQUIRE(std::holds_alternative<DCC::Commands::LoadImage>(cmd->command));
    }

    SECTION("ResetData command") {
        auto cmd = DCC::resetData();
        REQUIRE(std::holds_alternative<DCC::Commands::ResetData>(cmd->command));
    }

    SECTION("SetPerformanceMode command") {
        auto cmd = DCC::setPerformanceMode(DCC::PerformanceMode::OPTIMIZE_SPEED);
        REQUIRE(std::holds_alternative<DCC::Commands::SetPerformanceMode>(cmd->command));
        auto& c = std::get<DCC::Commands::SetPerformanceMode>(cmd->command);
        REQUIRE(c.performanceMode == DCC::PerformanceMode::OPTIMIZE_SPEED);
    }

    SECTION("ApplyCalibration command") {
        dai::CalibrationHandler calHandler;  // Assuming default constructible
        auto cmd = DCC::applyCalibration(calHandler);
        REQUIRE(std::holds_alternative<DCC::Commands::ApplyCalibration>(cmd->command));
        auto& c = std::get<DCC::Commands::ApplyCalibration>(cmd->command);
        // Optionally verify that calibration matches (if operator== is defined)
    }
}
