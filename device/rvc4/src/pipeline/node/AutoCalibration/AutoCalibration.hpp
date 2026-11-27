#pragma once

#include <memory>
#include <utility>

#include "depthai/pipeline/node/AutoCalibration.hpp"

namespace dai {
namespace gate {

class AutoCalibration : public ::dai::node::AutoCalibration {
   public:
    constexpr static const char* NAME = ::dai::node::AutoCalibration::NAME;

    AutoCalibration() : AutoCalibration(std::make_unique<Properties>()) {}
    AutoCalibration(std::unique_ptr<Properties> props) : ::dai::node::AutoCalibration(std::move(props)) {}

    bool runOnHost() const override {
        return true;  // Runs as a firmware thread on the RVC4 CPU
    }
};

}  // namespace gate
}  // namespace dai
