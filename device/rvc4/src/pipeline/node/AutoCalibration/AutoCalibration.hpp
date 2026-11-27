#pragma once

#include <memory>
#include <utility>

#include "depthai/pipeline/node/AutoCalibration.hpp"

namespace dai {
namespace gate {

/**
 * AutoCalibration running inside the RVC4 firmware. Drives the device-side DynamicCalibration node through
 * the same command and result queues the host version uses.
 */
class AutoCalibration : public ::dai::node::AutoCalibration {
   public:
    constexpr static const char* NAME = ::dai::node::AutoCalibration::NAME;

    AutoCalibration() : AutoCalibration(std::make_unique<Properties>()) {}
    AutoCalibration(std::unique_ptr<Properties> props) : ::dai::node::AutoCalibration(std::move(props)) {}

    bool runOnHost() const override {
        // Run as a firmware thread on the RVC4 ARM CPU.
        return true;
    }
};

}  // namespace gate
}  // namespace dai
