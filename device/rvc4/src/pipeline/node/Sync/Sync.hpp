#pragma once

#include "depthai/pipeline/node/Sync.hpp"
#include "pipeline/ThreadedNodeImpl.hpp"

namespace dai {
namespace gate {

class Sync : public dai::node::Sync {
   protected:
    TimestampSource getDefaultTimestampSource() const override {
        return TimestampSource::DEVICE;
    }

   public:
    std::shared_ptr<spdlog::async_logger>& logger = ThreadedNode::pimpl->logger;

    constexpr static const char* NAME = node::Sync::NAME;
    Sync() : Sync(std::make_unique<dai::node::Sync::Properties>()) {}
    Sync(std::unique_ptr<Properties> props) : dai::node::Sync(std::move(props)) {}
    bool runOnHost() const override {
        return true;
    }
};

}  // namespace gate
}  // namespace dai
