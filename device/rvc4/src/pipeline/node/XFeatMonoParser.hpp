#pragma once

// C++ std
#include <memory>

// libraries
#include <spdlog/async_logger.h>

#include "depthai/beta/node/XFeatMonoParser.hpp"
#include "pipeline/ThreadedNodeImpl.hpp"

namespace dai {
namespace gate {
class XFeatMonoParser : public beta::node::XFeatMonoParser {
   public:
    std::shared_ptr<spdlog::async_logger>& logger = ThreadedNode::pimpl->logger;
    constexpr static const char* NAME = beta::node::XFeatMonoParser::NAME;
    XFeatMonoParser() : XFeatMonoParser(std::make_unique<XFeatMonoParser::Properties>()) {}
    XFeatMonoParser(std::unique_ptr<Properties> props) : beta::node::XFeatMonoParser(std::move(props)) {}
    bool runOnHost() const override {
        return true;
    }
};

}  // namespace gate
}  // namespace dai
