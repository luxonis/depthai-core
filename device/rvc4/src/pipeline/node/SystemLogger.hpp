#pragma once

// libraries
#include "depthai/pipeline/node/SystemLogger.hpp"
#include "depthai/properties/SystemLoggerProperties.hpp"
#include "pipeline/ThreadedNodeImpl.hpp"

namespace dai {
namespace gate {

class SystemLogger : public dai::node::SystemLogger {
   public:
    std::shared_ptr<spdlog::async_logger>& logger = ThreadedNode::pimpl->logger;

    constexpr static const char* NAME = node::SystemLogger::NAME;
    SystemLogger() : SystemLogger(std::make_unique<dai::node::SystemLogger::Properties>()) {}
    SystemLogger(std::unique_ptr<Properties> props) : dai::node::SystemLogger(std::move(props)) {
        // override that this is a host node
    }
    bool runOnHost() const override {
        return true;
    }
    void run() override;
};

}  // namespace gate
}  // namespace dai
