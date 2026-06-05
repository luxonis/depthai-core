#pragma once

#include <depthai/pipeline/DeviceNode.hpp>
#include <depthai/properties/SplitterNodeProperties.hpp>

namespace dai {
namespace node {

/**
 * @brief SplitterNode skeleton.
 *
 * The splitting logic is intentionally left unimplemented.
 */
class SplitterNode : public DeviceNodeCRTP<DeviceNode, SplitterNode, SplitterNodeProperties>, public HostRunnable {
   public:
    constexpr static const char* NAME = "SplitterNode";
    using DeviceNodeCRTP::DeviceNodeCRTP;

    SplitterNode() = default;
    ~SplitterNode() override;

    /**
     * Input message to split.
     */
    Input input{*this, {"input", DEFAULT_GROUP, DEFAULT_BLOCKING, DEFAULT_QUEUE_SIZE, {{{DatatypeEnum::Buffer, true}}}}};

    /**
     * Splitter output message.
     */
    Output output{*this, {"output", DEFAULT_GROUP, {{{DatatypeEnum::Buffer, true}}}}};

    /**
     * Specify whether to run on host or device
     * By default, the node will run on device.
     */
    void setRunOnHost(bool runOnHost);

    /**
     * Check if the node is set to run on host
     */
    bool runOnHost() const override;

    void run() override;

   private:
    bool runOnHostVar = false;
};

}  // namespace node
}  // namespace dai
