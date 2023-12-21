#pragma once

#include <depthai/pipeline/DeviceNode.hpp>

// shared
#include <depthai/properties/XLinkInProperties.hpp>

namespace dai {
namespace node {

/**
 * @brief XLinkIn node. Receives messages over XLink.
 */
class XLinkIn : public NodeCRTP<DeviceNode, XLinkIn, XLinkInProperties> {
   public:
    constexpr static const char* NAME = "XLinkIn";
    using NodeCRTP::NodeCRTP;

   public:
    virtual ~XLinkIn() = default;

    /**
     * Outputs message of same type as send from host.
     */
    Output out{true, *this, "out", Output::Type::MSender, {{DatatypeEnum::Buffer, true}}};

    /**
     * Specifies XLink stream name to use.
     *
     * The name should not start with double underscores '__',
     * as those are reserved for internal use.
     * @param name Stream name
     */
    void setStreamName(const std::string& name);

    /**
     * Set maximum message size it can receive
     * @param maxDataSize Maximum size in bytes
     */
    void setMaxDataSize(std::uint32_t maxDataSize);

    /**
     * Set number of frames in pool for sending messages forward
     * @param numFrames Maximum number of frames in pool
     */
    void setNumFrames(std::uint32_t numFrames);

    /// Get stream name
    std::string getStreamName() const;
    /// Get maximum messages size in bytes
    std::uint32_t getMaxDataSize() const;
    /// Get number of frames in pool
    std::uint32_t getNumFrames() const;
};

}  // namespace node
}  // namespace dai
