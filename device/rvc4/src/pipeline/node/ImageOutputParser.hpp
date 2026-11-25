#pragma once

// C++ std
#include <memory>

// libraries
#include <spdlog/async_logger.h>

#include "depthai/beta/node/ImageOutputParser.hpp"
#include "pipeline/ThreadedNodeImpl.hpp"

namespace dai {
namespace gate {
class ImageOutputParser : public beta::node::ImageOutputParser {
   public:
    std::shared_ptr<spdlog::async_logger>& logger = ThreadedNode::pimpl->logger;
    constexpr static const char* NAME = beta::node::ImageOutputParser::NAME;
    ImageOutputParser() : ImageOutputParser(std::make_unique<ImageOutputParser::Properties>()) {}
    ImageOutputParser(std::unique_ptr<Properties> props) : beta::node::ImageOutputParser(std::move(props)) {}
    bool runOnHost() const override {
        return true;
    }
};

}  // namespace gate
}  // namespace dai
