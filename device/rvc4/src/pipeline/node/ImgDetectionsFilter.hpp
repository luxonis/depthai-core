#pragma once

#include <memory>

#include <spdlog/async_logger.h>

#include "depthai/pipeline/node/ImgDetectionsFilter.hpp"
#include "pipeline/ThreadedNodeImpl.hpp"

namespace dai {
namespace gate {
class ImgDetectionsFilter : public node::ImgDetectionsFilter {
   public:
    std::shared_ptr<spdlog::async_logger>& logger = ThreadedNode::pimpl->logger;
    constexpr static const char* NAME = node::ImgDetectionsFilter::NAME;
    ImgDetectionsFilter() : ImgDetectionsFilter(std::make_unique<ImgDetectionsFilter::Properties>()) {}
    explicit ImgDetectionsFilter(std::unique_ptr<Properties> props) : node::ImgDetectionsFilter(std::move(props)) {}
    bool runOnHost() const override {
        return true;
    }
};

}  // namespace gate
}  // namespace dai
