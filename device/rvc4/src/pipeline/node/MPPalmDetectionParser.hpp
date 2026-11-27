#pragma once

#include <memory>

#include <spdlog/async_logger.h>

#include "depthai/beta/node/MPPalmDetectionParser.hpp"
#include "pipeline/ThreadedNodeImpl.hpp"

namespace dai {
namespace gate {
class MPPalmDetectionParser : public beta::node::MPPalmDetectionParser {
   public:
    std::shared_ptr<spdlog::async_logger>& logger = ThreadedNode::pimpl->logger;
    constexpr static const char* NAME = beta::node::MPPalmDetectionParser::NAME;
    MPPalmDetectionParser() : MPPalmDetectionParser(std::make_unique<MPPalmDetectionParser::Properties>()) {}
    MPPalmDetectionParser(std::unique_ptr<Properties> props) : beta::node::MPPalmDetectionParser(std::move(props)) {}
    bool runOnHost() const override {
        return true;
    }
};

}  // namespace gate
}  // namespace dai
