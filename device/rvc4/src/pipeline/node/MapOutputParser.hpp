#pragma once

// C++ std
#include <memory>

// libraries
#include <spdlog/async_logger.h>

#include "depthai/beta/node/MapOutputParser.hpp"
#include "pipeline/ThreadedNodeImpl.hpp"

namespace dai {
namespace gate {
class MapOutputParser : public beta::node::MapOutputParser {
   public:
    std::shared_ptr<spdlog::async_logger>& logger = ThreadedNode::pimpl->logger;
    constexpr static const char* NAME = beta::node::MapOutputParser::NAME;
    MapOutputParser() : MapOutputParser(std::make_unique<MapOutputParser::Properties>()) {}
    MapOutputParser(std::unique_ptr<Properties> props) : beta::node::MapOutputParser(std::move(props)) {}
    bool runOnHost() const override {
        return true;
    }
};

}  // namespace gate
}  // namespace dai
