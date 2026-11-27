#pragma once

#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "depthai/pipeline/node/DynamicCalibrationNode.hpp"

namespace dai {
namespace gate {

/**
 * Dynamic calibration running inside the RVC4 firmware. The core node does all the work; this subclass only
 * replaces the host dai::Device accessors with the camera discovery and the EEPROM.
 */
class DynamicCalibration : public ::dai::node::DynamicCalibration {
   public:
    constexpr static const char* NAME = ::dai::node::DynamicCalibration::NAME;

    DynamicCalibration() : DynamicCalibration(std::make_unique<Properties>()) {}
    DynamicCalibration(std::unique_ptr<Properties> props) : ::dai::node::DynamicCalibration(std::move(props)) {}

    bool runOnHost() const override {
        // Run as a firmware thread on the RVC4 ARM CPU.
        return true;
    }

   protected:
    CalibrationHandler readFactoryCalibration() override;
    std::vector<StereoPair> getStereoPairs() override;
    std::pair<std::string, std::string> getProductAndBoardName() override;
    void applyCalibration(const CalibrationHandler& handler, bool flash) override;
};

}  // namespace gate
}  // namespace dai
