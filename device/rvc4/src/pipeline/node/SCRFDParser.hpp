#pragma once

#include <memory>

#include <spdlog/async_logger.h>

#include "depthai/beta/node/SCRFDParser.hpp"
#include "pipeline/ThreadedNodeImpl.hpp"

namespace dai {
namespace gate {
class SCRFDParser : public beta::node::SCRFDParser {
   public:
    std::shared_ptr<spdlog::async_logger>& logger = ThreadedNode::pimpl->logger;
    constexpr static const char* NAME = beta::node::SCRFDParser::NAME;
    SCRFDParser() : SCRFDParser(std::make_unique<SCRFDParser::Properties>()) {}
    SCRFDParser(std::unique_ptr<Properties> props) : beta::node::SCRFDParser(std::move(props)) {}
    bool runOnHost() const override {
        return true;
    }
};

}  // namespace gate
}  // namespace dai
