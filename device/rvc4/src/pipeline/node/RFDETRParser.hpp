#pragma once

// C++ std
#include <memory>

// libraries
#include <spdlog/async_logger.h>

#include "depthai/beta/node/RFDETRParser.hpp"
#include "pipeline/ThreadedNodeImpl.hpp"

namespace dai {
namespace gate {
class RFDETRParser : public beta::node::RFDETRParser {
   public:
    std::shared_ptr<spdlog::async_logger>& logger = ThreadedNode::pimpl->logger;
    constexpr static const char* NAME = beta::node::RFDETRParser::NAME;
    RFDETRParser() : RFDETRParser(std::make_unique<RFDETRParser::Properties>()) {}
    RFDETRParser(std::unique_ptr<Properties> props) : beta::node::RFDETRParser(std::move(props)) {}
    bool runOnHost() const override {
        return true;
    }
};

}  // namespace gate
}  // namespace dai
