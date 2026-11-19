#pragma once

// libraries
#include "depthai/pipeline/node/internal/XLinkIn.hpp"

// shared
#include "pipeline/ThreadedNodeImpl.hpp"

namespace dai {
namespace gate {

class XLinkIn : public node::internal::XLinkIn {
   public:
    std::shared_ptr<spdlog::async_logger>& logger = ThreadedNode::pimpl->logger;

    constexpr static const char* NAME = node::internal::XLinkIn::NAME;

   public:
    XLinkIn() : XLinkIn(std::make_unique<XLinkIn::Properties>()) {}
    XLinkIn(std::unique_ptr<Properties> props) : node::internal::XLinkIn(std::move(props)) {
        // override that this is a host node
    }
    bool runOnHost() const override {
        return true;
    }

    void run() override;
};

}  // namespace gate
}  // namespace dai
