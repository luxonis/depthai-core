#pragma once

#include <memory>

#include <spdlog/async_logger.h>

#include "depthai/beta/node/PPTextDetectionParser.hpp"
#include "pipeline/ThreadedNodeImpl.hpp"

namespace dai {
namespace gate {
class PPTextDetectionParser : public beta::node::PPTextDetectionParser {
   public:
    std::shared_ptr<spdlog::async_logger>& logger = ThreadedNode::pimpl->logger;
    constexpr static const char* NAME = beta::node::PPTextDetectionParser::NAME;
    PPTextDetectionParser() : PPTextDetectionParser(std::make_unique<PPTextDetectionParser::Properties>()) {}
    PPTextDetectionParser(std::unique_ptr<Properties> props) : beta::node::PPTextDetectionParser(std::move(props)) {}
    bool runOnHost() const override {
        return true;
    }
};

}  // namespace gate
}  // namespace dai
