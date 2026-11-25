#pragma once

// C++ std
#include <memory>

// libraries
#include <spdlog/async_logger.h>

#include "depthai/beta/node/LaneDetectionParser.hpp"
#include "pipeline/ThreadedNodeImpl.hpp"

namespace dai {
namespace gate {
class LaneDetectionParser : public beta::node::LaneDetectionParser {
   public:
    std::shared_ptr<spdlog::async_logger>& logger = ThreadedNode::pimpl->logger;
    constexpr static const char* NAME = beta::node::LaneDetectionParser::NAME;
    LaneDetectionParser() : LaneDetectionParser(std::make_unique<LaneDetectionParser::Properties>()) {}
    LaneDetectionParser(std::unique_ptr<Properties> props) : beta::node::LaneDetectionParser(std::move(props)) {}
    bool runOnHost() const override {
        return true;
    }
};

}  // namespace gate
}  // namespace dai
