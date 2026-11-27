#pragma once

#include "depthai/pipeline/node/Rectification.hpp"
#include "pipeline/ThreadedNodeImpl.hpp"

namespace dai {
namespace gate {

class Rectification : public dai::node::Rectification {
   public:
    std::shared_ptr<spdlog::async_logger>& logger = ThreadedNode::pimpl->logger;

    constexpr static const char* NAME = node::Rectification::NAME;
    Rectification() : Rectification(std::make_unique<dai::node::Rectification::Properties>()) {}
    Rectification(std::unique_ptr<Properties> props) : dai::node::Rectification(std::move(props)) {}
    bool runOnHost() const override {
        return true;
    }
};


}  // namespace gate
}  // namespace dai
