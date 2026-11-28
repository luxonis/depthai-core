#pragma once

#include <memory>
#include <utility>

#include "depthai/pipeline/node/Stitching.hpp"

namespace dai {
namespace gate {

class Stitching : public ::dai::node::Stitching {
   public:
    constexpr static const char* NAME = ::dai::node::Stitching::NAME;

    Stitching() : Stitching(std::make_unique<Properties>()) {}
    Stitching(std::unique_ptr<Properties> props) : ::dai::node::Stitching(std::move(props)) {}

    bool runOnHost() const override {
        return true;  // Runs as a firmware thread (ARM CPU)
    }
};

}  // namespace gate
}  // namespace dai
