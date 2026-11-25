#pragma once

#include <memory>

#include "depthai/pipeline/node/GPUStereo.hpp"
#include "pipeline/ThreadedNodeImpl.hpp"

namespace dai {
namespace gate {

class GPUStereo : public ::dai::node::GPUStereo {
    std::shared_ptr<spdlog::async_logger>& logger = ThreadedNode::pimpl->logger;

   public:
    constexpr static const char* NAME = node::GPUStereo::NAME;

    GPUStereo() : GPUStereo(std::make_unique<dai::node::GPUStereo::Properties>()) {}
    GPUStereo(std::unique_ptr<Properties> props) : dai::node::GPUStereo(std::move(props)) {}

    bool runOnHost() const override {
        return true;
    }

    void run() override;
};

}  // namespace gate
}  // namespace dai
