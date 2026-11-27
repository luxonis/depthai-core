#pragma once

#include <memory>

#include <spdlog/async_logger.h>

#include "depthai/beta/node/ClassificationParser.hpp"
#include "pipeline/ThreadedNodeImpl.hpp"

namespace dai {
namespace gate {
class ClassificationParser : public beta::node::ClassificationParser {
   public:
    std::shared_ptr<spdlog::async_logger>& logger = ThreadedNode::pimpl->logger;
    constexpr static const char* NAME = beta::node::ClassificationParser::NAME;
    ClassificationParser() : ClassificationParser(std::make_unique<ClassificationParser::Properties>()) {}
    ClassificationParser(std::unique_ptr<Properties> props) : beta::node::ClassificationParser(std::move(props)) {}
    bool runOnHost() const override {
        return true;
    }
};

}  // namespace gate
}  // namespace dai
