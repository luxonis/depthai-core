#pragma once

// C++ std
#include <memory>

// libraries
#include <spdlog/async_logger.h>

#include "depthai/beta/node/SuperAnimalParser.hpp"
#include "pipeline/ThreadedNodeImpl.hpp"

namespace dai {
namespace gate {
class SuperAnimalParser : public beta::node::SuperAnimalParser {
   public:
    std::shared_ptr<spdlog::async_logger>& logger = ThreadedNode::pimpl->logger;
    constexpr static const char* NAME = beta::node::SuperAnimalParser::NAME;
    SuperAnimalParser() : SuperAnimalParser(std::make_unique<SuperAnimalParser::Properties>()) {}
    SuperAnimalParser(std::unique_ptr<Properties> props) : beta::node::SuperAnimalParser(std::move(props)) {}
    bool runOnHost() const override {
        return true;
    }
};

}  // namespace gate
}  // namespace dai
