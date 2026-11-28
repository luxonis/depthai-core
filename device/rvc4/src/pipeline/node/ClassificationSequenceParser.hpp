#pragma once

#include <memory>

#include <spdlog/async_logger.h>

#include "depthai/beta/node/ClassificationSequenceParser.hpp"
#include "pipeline/ThreadedNodeImpl.hpp"

namespace dai {
namespace gate {
class ClassificationSequenceParser : public beta::node::ClassificationSequenceParser {
   public:
    std::shared_ptr<spdlog::async_logger>& logger = ThreadedNode::pimpl->logger;
    constexpr static const char* NAME = beta::node::ClassificationSequenceParser::NAME;
    ClassificationSequenceParser() : ClassificationSequenceParser(std::make_unique<ClassificationSequenceParser::Properties>()) {}
    ClassificationSequenceParser(std::unique_ptr<Properties> props) : beta::node::ClassificationSequenceParser(std::move(props)) {}
    bool runOnHost() const override {
        return true;
    }
};

}  // namespace gate
}  // namespace dai
