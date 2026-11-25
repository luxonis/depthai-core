#pragma once

// libraries
#include "depthai/pipeline/node/Warp.hpp"
#include "pipeline/ThreadedNodeImpl.hpp"
#include "pipeline/node/Warp/rvc4/WarpDispatcherRvc4.hpp"

namespace dai {
namespace gate {
class Warp : public ::dai::node::Warp {
    std::shared_ptr<spdlog::async_logger>& logger = ThreadedNode::pimpl->logger;

    std::unique_ptr<WarpDispatcher> warpDispatcher;
    dai::WarpProperties warpConfig;

    // RawWarpConfig getConfigFromProperties(const dai::WarpProperties& cfg);

   public:
    constexpr static const char* NAME = node::Warp::NAME;
    Warp() : Warp(std::make_unique<dai::node::Warp::Properties>()) {}
    Warp(std::unique_ptr<Properties> props) : dai::node::Warp(std::move(props)) {
        // override that this is a host node
    }
    bool runOnHost() const override {
        return true;
    }
    void run() override;
};

}  // namespace gate
}  // namespace dai
