#pragma once

#include <depthai/pipeline/DeviceNode.hpp>

// standard
#include <fstream>

// shared
#include <depthai/properties/SyncProperties.hpp>

namespace dai {
namespace node {

/**
 * @brief Sync node. Performs syncing between image frames
 */
class Sync : public DeviceNodeCRTP<DeviceNode, Sync, SyncProperties>, public HostRunnable {
   private:
    bool runOnHostVar = false;

   public:
    constexpr static const char* NAME = "Sync";
    using DeviceNodeCRTP::DeviceNodeCRTP;

    /**
     * A map of inputs
     */
    InputMap inputs{*this, "inputs", {"", DEFAULT_GROUP, false, 10, {{{DatatypeEnum::Buffer, true}}}, DEFAULT_WAIT_FOR_MESSAGE}};

    /**
     * Output message of type MessageGroup
     */
    // Output out{*this, "out", Output::Type::MSender, {{DatatypeEnum::MessageGroup, false}}};
    Output out{*this, {"out", DEFAULT_GROUP, {{{DatatypeEnum::MessageGroup, false}}}}};

    /**
     * Set the maximal interval between messages in the group
     * @param syncThreshold Maximal interval between messages in the group
     */
    void setSyncThreshold(std::chrono::nanoseconds syncThreshold);

    /**
     * Set the number of attempts to get the specified max interval between messages in the group
     * @param syncAttempts Number of attempts to get the specified max interval between messages in the group:
     *   - if syncAttempts = 0 then the node sends a message as soon at the group is filled
     *   - if syncAttempts > 0 then the node will make syncAttemts attempts to synchronize before sending out a message
     *   - if syncAttempts = -1 (default) then the node will only send a message if successfully synchronized
     */
    void setSyncAttempts(int syncAttempts);

    /**
     * Gets the maximal interval between messages in the group in milliseconds
     */
    std::chrono::nanoseconds getSyncThreshold() const;

    /**
     * Gets the number of sync attempts
     */
    int getSyncAttempts() const;

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
};

}  // namespace node
}  // namespace dai
