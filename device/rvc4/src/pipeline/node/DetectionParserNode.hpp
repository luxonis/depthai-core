#pragma once

#include <memory>
#include <vector>

#include <spdlog/async_logger.h>

#include "depthai/pipeline/node/DetectionParser.hpp"
#include "depthai/properties/DetectionParserProperties.hpp"
#include "pipeline/ThreadedNodeImpl.hpp"

namespace dai {
namespace gate {
class DetectionParser : public node::DetectionParser {
   public:
    std::shared_ptr<spdlog::async_logger>& logger = ThreadedNode::pimpl->logger;
    constexpr static const char* NAME = node::DetectionParser::NAME;
    DetectionParser() : DetectionParser(std::make_unique<DetectionParser::Properties>()) {}
    DetectionParser(std::unique_ptr<Properties> props) : node::DetectionParser(std::move(props)) {}
    bool runOnHost() const override {
        return true;
    }
};

}  // namespace gate
}  // namespace dai