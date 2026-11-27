#pragma once

#include <atomic>

#include "depthai/pipeline/node/Script.hpp"
#include "pipeline/ThreadedNodeImpl.hpp"

namespace dai {
namespace gate {

class Script : public dai::node::Script {
   public:
   std::shared_ptr<spdlog::async_logger>& logger = ThreadedNode::pimpl->logger;

    constexpr static const char* NAME = node::Script::NAME;
    Script() : Script(std::make_unique<dai::node::Script::Properties>()) {}
    Script(std::unique_ptr<Properties> props) : dai::node::Script(std::move(props)) {}
    bool runOnHost() const override { return true; }
    void run() override;

    void buildStage1() override;

    private:
};

}  // namespace gate
}  // namespace dai
