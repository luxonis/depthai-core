#pragma once

// C++ std
#include <memory>

// libraries
#include <spdlog/async_logger.h>

#include "depthai/pipeline/node/SegmentationParser.hpp"
#include "depthai/properties/SegmentationParserProperties.hpp"
#include "pipeline/ThreadedNodeImpl.hpp"

namespace dai {
namespace gate {
class SegmentationParser : public node::SegmentationParser {
   public:
    std::shared_ptr<spdlog::async_logger>& logger = ThreadedNode::pimpl->logger;
    constexpr static const char* NAME = node::SegmentationParser::NAME;
    SegmentationParser() : SegmentationParser(std::make_unique<SegmentationParser::Properties>()) {}
    SegmentationParser(std::unique_ptr<Properties> props) : node::SegmentationParser(std::move(props)) {}
    bool runOnHost() const override {
        return true;
    }

    std::shared_ptr<SegmentationParserConfig> inConfig;
};

}  // namespace gate
}  // namespace dai