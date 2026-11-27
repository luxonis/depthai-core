#pragma once

// C++ std
#include <memory>

// libraries
#include <spdlog/async_logger.h>

#include "depthai/beta/node/EmbeddingsParser.hpp"
#include "pipeline/ThreadedNodeImpl.hpp"

namespace dai {
namespace gate {
class EmbeddingsParser : public beta::node::EmbeddingsParser {
   public:
    std::shared_ptr<spdlog::async_logger>& logger = ThreadedNode::pimpl->logger;
    constexpr static const char* NAME = beta::node::EmbeddingsParser::NAME;
    EmbeddingsParser() : EmbeddingsParser(std::make_unique<EmbeddingsParser::Properties>()) {}
    EmbeddingsParser(std::unique_ptr<Properties> props) : beta::node::EmbeddingsParser(std::move(props)) {}
    bool runOnHost() const override {
        return true;
    }
};

}  // namespace gate
}  // namespace dai
