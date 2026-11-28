#pragma once

#include <XLink/XLinkPublicDefines.h>
#include <spdlog/async_logger.h>

#include "depthai/pipeline/datatype/Buffer.hpp"
#include "depthai/pipeline/datatype/ImgFrame.hpp"
#include "depthai/pipeline/node/internal/XLinkOut.hpp"
#include "pipeline/ThreadedNodeImpl.hpp"
namespace dai {
namespace gate {

class XLinkOutTestable;

class XLinkOut : public dai::node::internal::XLinkOut {
   public:
    std::shared_ptr<spdlog::async_logger>& logger = ThreadedNode::pimpl->logger;
    constexpr static const char* NAME = node::internal::XLinkOut::NAME;

   public:
    XLinkOut() : XLinkOut(std::make_unique<XLinkOut::Properties>()) {}
    XLinkOut(std::unique_ptr<Properties> props) : dai::node::internal::XLinkOut(std::move(props)) {}
    bool runOnHost() const override {
        return true;
    }
    void run() override;

   private:
    virtual XLinkError_t writeData(streamId_t streamId, dai::span<const uint8_t> data, dai::span<const uint8_t> metadata) const;

    virtual XLinkError_t writeData(streamId_t streamId, dai::span<const uint8_t> metadata) const;

    virtual XLinkError_t writeData(streamId_t streamId, const uint8_t* data, size_t size) const;

    XLinkError_t sendPacketizedFrame(streamId_t streamId, dai::span<const uint8_t> data, dai::span<const uint8_t> metadata) const;

    bool shouldSkipFrame(double maxFps,
                         std::chrono::steady_clock::time_point& tPrevPacket,
                         std::chrono::steady_clock::time_point& tPrevSecond,
                         int& packetCounter) const;

    XLinkError_t transmitMessage(streamId_t& streamId, std::shared_ptr<dai::ADatatype> message, size_t& currentStreamSize, const std::string& streamName) const;

    void ensureStreamSize(streamId_t& streamId, size_t& currentSize, size_t requiredSize, const std::string& name) const;

    bool fpsLimitEnabled = false;

    friend XLinkOutTestable;
};

}  // namespace gate
}  // namespace dai
