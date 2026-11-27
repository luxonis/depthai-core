#pragma once

// libraries
#include "depthai/pipeline/node/ObjectTracker.hpp"

namespace dai {
namespace gate {

class ObjectTracker : public ::dai::node::ObjectTracker {
   public:
    constexpr static const char* NAME = node::ObjectTracker::NAME;
    ObjectTracker() : ObjectTracker(std::make_unique<dai::node::ObjectTracker::Properties>()) {}
    ObjectTracker(std::unique_ptr<Properties> props) : dai::node::ObjectTracker(std::move(props)) {
        // override that this is a host node
    }
    bool runOnHost() const override {
        return true;
    }
};

}  // namespace gate
}  // namespace dai
