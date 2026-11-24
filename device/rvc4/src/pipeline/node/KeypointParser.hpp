#pragma once

// C++ std
#include <memory>

// libraries
#include <spdlog/async_logger.h>

#include "depthai/beta/node/KeypointParser.hpp"
#include "pipeline/ThreadedNodeImpl.hpp"

namespace dai {
namespace gate {
class KeypointParser : public beta::node::KeypointParser {
   public:
    std::shared_ptr<spdlog::async_logger>& logger = ThreadedNode::pimpl->logger;
    constexpr static const char* NAME = beta::node::KeypointParser::NAME;
    KeypointParser() : KeypointParser(std::make_unique<KeypointParser::Properties>()) {}
    KeypointParser(std::unique_ptr<Properties> props) : beta::node::KeypointParser(std::move(props)) {}
    bool runOnHost() const override {
        return true;
    }
};

}  // namespace gate
}  // namespace dai
