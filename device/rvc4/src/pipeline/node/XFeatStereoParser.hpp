#pragma once

#include <memory>

#include <spdlog/async_logger.h>

#include "depthai/beta/node/XFeatStereoParser.hpp"
#include "pipeline/ThreadedNodeImpl.hpp"

namespace dai {
namespace gate {
class XFeatStereoParser : public beta::node::XFeatStereoParser {
   public:
    std::shared_ptr<spdlog::async_logger>& logger = ThreadedNode::pimpl->logger;
    constexpr static const char* NAME = beta::node::XFeatStereoParser::NAME;
    XFeatStereoParser() : XFeatStereoParser(std::make_unique<XFeatStereoParser::Properties>()) {}
    XFeatStereoParser(std::unique_ptr<Properties> props) : beta::node::XFeatStereoParser(std::move(props)) {}
    bool runOnHost() const override {
        return true;
    }
};

}  // namespace gate
}  // namespace dai
