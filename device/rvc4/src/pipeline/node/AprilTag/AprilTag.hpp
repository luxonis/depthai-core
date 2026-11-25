#pragma once

// libraries
#include "depthai/pipeline/node/AprilTag.hpp"
#include "pipeline/ThreadedNodeImpl.hpp"

namespace dai {
namespace gate {
class AprilTag : public ::dai::node::AprilTag {
   public:
    std::shared_ptr<spdlog::async_logger>& logger = ThreadedNode::pimpl->logger;

    constexpr static const char* NAME = node::AprilTag::NAME;
    AprilTag() : AprilTag(std::make_unique<dai::node::AprilTag::Properties>()) {}
    AprilTag(std::unique_ptr<Properties> props) : dai::node::AprilTag(std::move(props)) {}
    bool runOnHost() const override {
        return true;
    }
};

}  // namespace gate
}  // namespace dai