#pragma once

#include <memory>

#include <spdlog/async_logger.h>

#include "depthai/beta/node/HRNetParser.hpp"
#include "pipeline/ThreadedNodeImpl.hpp"

namespace dai {
namespace gate {
class HRNetParser : public beta::node::HRNetParser {
   public:
    std::shared_ptr<spdlog::async_logger>& logger = ThreadedNode::pimpl->logger;
    constexpr static const char* NAME = beta::node::HRNetParser::NAME;
    HRNetParser() : HRNetParser(std::make_unique<HRNetParser::Properties>()) {}
    HRNetParser(std::unique_ptr<Properties> props) : beta::node::HRNetParser(std::move(props)) {}
    bool runOnHost() const override {
        return true;
    }
};

}  // namespace gate
}  // namespace dai
