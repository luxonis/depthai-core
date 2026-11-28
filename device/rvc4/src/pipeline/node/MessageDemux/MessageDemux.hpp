#pragma once

#include "depthai/pipeline/node/MessageDemux.hpp"
#include "depthai/properties/MessageDemuxProperties.hpp"
#include "pipeline/ThreadedNodeImpl.hpp"

namespace dai {
namespace gate {

class MessageDemux : public dai::node::MessageDemux {
   public:
    std::shared_ptr<spdlog::async_logger>& logger = ThreadedNode::pimpl->logger;

    constexpr static const char* NAME = node::MessageDemux::NAME;
    MessageDemux() : MessageDemux(std::make_unique<dai::node::MessageDemux::Properties>()) {}
    MessageDemux(std::unique_ptr<Properties> props) : dai::node::MessageDemux(std::move(props)) {}
    bool runOnHost() const override {
        return true;
    }
};

}  // namespace gate
}  // namespace dai
