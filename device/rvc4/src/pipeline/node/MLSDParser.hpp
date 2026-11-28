#pragma once

#include <memory>

#include <spdlog/async_logger.h>

#include "depthai/beta/node/MLSDParser.hpp"
#include "pipeline/ThreadedNodeImpl.hpp"

namespace dai {
namespace gate {
class MLSDParser : public beta::node::MLSDParser {
   public:
    std::shared_ptr<spdlog::async_logger>& logger = ThreadedNode::pimpl->logger;
    constexpr static const char* NAME = beta::node::MLSDParser::NAME;
    MLSDParser() : MLSDParser(std::make_unique<MLSDParser::Properties>()) {}
    MLSDParser(std::unique_ptr<Properties> props) : beta::node::MLSDParser(std::move(props)) {}
    bool runOnHost() const override {
        return true;
    }
};

}  // namespace gate
}  // namespace dai
