#pragma once

#include "depthai/pipeline/datatype/GateControl.hpp"
#include "depthai/pipeline/node/Gate.hpp"

namespace dai {
namespace gate {

class Gate : public ::dai::node::Gate {
   public:
    constexpr static const char* NAME = node::Gate::NAME;

    Gate() : Gate(std::make_unique<dai::node::Gate::Properties>()) {}

    Gate(std::unique_ptr<GateProperties> props) : dai::node::Gate(std::move(props)) {}

    bool runOnHost() const override {
        return true;
    }
};
}  // namespace gate
}  // namespace dai
