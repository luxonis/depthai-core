#pragma once

// C++ std
#include <memory>

// libraries
#include <spdlog/async_logger.h>

#include "depthai/beta/node/RegressionParser.hpp"
#include "pipeline/ThreadedNodeImpl.hpp"

namespace dai {
namespace gate {
class RegressionParser : public beta::node::RegressionParser {
   public:
    std::shared_ptr<spdlog::async_logger>& logger = ThreadedNode::pimpl->logger;
    constexpr static const char* NAME = beta::node::RegressionParser::NAME;
    RegressionParser() : RegressionParser(std::make_unique<RegressionParser::Properties>()) {}
    RegressionParser(std::unique_ptr<Properties> props) : beta::node::RegressionParser(std::move(props)) {}
    bool runOnHost() const override {
        return true;
    }
};

}  // namespace gate
}  // namespace dai
