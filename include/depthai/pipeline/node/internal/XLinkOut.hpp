#pragma once

#include <depthai/pipeline/DeviceNode.hpp>

// shared
#include <depthai/properties/internal/XLinkOutProperties.hpp>

namespace dai {
namespace node {
namespace internal {

using XLinkOutProperties = ::dai::internal::XLinkOutProperties;

/**
 * @brief XLinkOut node. Sends messages over XLink.
 */
class XLinkOut : public DeviceNodeCRTP<DeviceNode, XLinkOut, XLinkOutProperties> {
   public:
    constexpr static const char* NAME = "XLinkOut";
    using DeviceNodeCRTP::DeviceNodeCRTP;

   public:
    /**
     * Input for any type of messages to be transferred over XLink stream
     *
     * Default queue is blocking with size 8
     */
    // Input input{*this, "in", Input::Type::SReceiver, true, 8, {{DatatypeEnum::Buffer, true}}};
    Input input{*this, {"in", DEFAULT_GROUP, DEFAULT_BLOCKING, DEFAULT_QUEUE_SIZE, {{{DatatypeEnum::Buffer, true}}}, DEFAULT_WAIT_FOR_MESSAGE}};

    /**
     * Specifies XLink stream name to use.
     *
     * The name should not start with double underscores '__',
     * as those are reserved for internal use.
     * @param name Stream name
     */
    void setStreamName(const std::string& name);

    /**
     * Specifies a message sending limit. It's approximated from specified rate.
     *
     * @param fps Approximate rate limit in messages per second
     */
    void setFpsLimit(float fps);

    /**
     * Specify whether to transfer only messages attributes and not buffer data
     */
    void setMetadataOnly(bool metadataOnly);

    /// Get stream name
    std::string getStreamName() const;
    /// Get rate limit in messages per second
    float getFpsLimit() const;
    /// Get whether to transfer only messages attributes and not buffer data
    bool getMetadataOnly() const;

    void buildInternal() override;
};

}  // namespace internal
}  // namespace node
}  // namespace dai
