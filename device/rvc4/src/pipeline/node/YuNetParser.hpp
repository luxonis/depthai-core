#pragma once

// C++ std
#include <memory>

// libraries
#include <spdlog/async_logger.h>

#include "depthai/beta/node/YuNetParser.hpp"
#include "pipeline/ThreadedNodeImpl.hpp"

namespace dai {
namespace gate {
class YuNetParser : public beta::node::YuNetParser {
   public:
    std::shared_ptr<spdlog::async_logger>& logger = ThreadedNode::pimpl->logger;
    constexpr static const char* NAME = beta::node::YuNetParser::NAME;
    YuNetParser() : YuNetParser(std::make_unique<YuNetParser::Properties>()) {}
    YuNetParser(std::unique_ptr<Properties> props) : beta::node::YuNetParser(std::move(props)) {}
    bool runOnHost() const override {
        return true;
    }
};

}  // namespace gate
}  // namespace dai
