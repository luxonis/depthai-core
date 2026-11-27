#pragma once

#include "depthai/pipeline/node/BenchmarkIn.hpp"
#include "depthai/pipeline/node/BenchmarkOut.hpp"
#include "pipeline/ThreadedNodeImpl.hpp"

namespace dai {
namespace gate {

class BenchmarkOut : public dai::node::BenchmarkOut {
   public:
    std::shared_ptr<spdlog::async_logger>& logger = ThreadedNode::pimpl->logger;

    constexpr static const char* NAME = node::BenchmarkOut::NAME;
    BenchmarkOut() : BenchmarkOut(std::make_unique<dai::node::BenchmarkOut::Properties>()) {}
    BenchmarkOut(std::unique_ptr<Properties> props) : dai::node::BenchmarkOut(std::move(props)) {}
    bool runOnHost() const override {
        return true;
    }
};

class BenchmarkIn : public dai::node::BenchmarkIn {
   public:
    std::shared_ptr<spdlog::async_logger>& logger = ThreadedNode::pimpl->logger;

    constexpr static const char* NAME = node::BenchmarkIn::NAME;
    BenchmarkIn() : BenchmarkIn(std::make_unique<dai::node::BenchmarkIn::Properties>()) {}
    BenchmarkIn(std::unique_ptr<Properties> props) : dai::node::BenchmarkIn(std::move(props)) {}
    bool runOnHost() const override {
        return true;
    }
};

}  // namespace gate
}  // namespace dai
