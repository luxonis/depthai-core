#pragma once

#include <depthai/pipeline/DeviceNode.hpp>

// standard
#include <fstream>

// shared
#include <depthai/properties/SyncProperties.hpp>

namespace dai {
namespace node {

enum class SyncTimestamp { Steady, System };

template<SyncTimestamp TS>
struct SyncName {
    static constexpr const char* value = "Sync";
};

template<>
struct SyncName<SyncTimestamp::System> {
    static constexpr const char* value = "SyncSystem";
};

/**
 * @brief Sync node. Performs syncing between image frames
 */
template <SyncTimestamp TS>
class SyncBase : public DeviceNodeCRTP<DeviceNode, SyncBase<TS>, SyncProperties>, public HostRunnable {
   private:
    bool runOnHostVar = false;

   public:
    constexpr static const char* NAME = SyncName<TS>::value;
    using DeviceNodeCRTP<DeviceNode, SyncBase<TS>, SyncProperties>::DeviceNodeCRTP;

    /**
     * A map of inputs
     */
    DeviceNode::InputMap inputs{*this, "inputs", {"", DeviceNode::DEFAULT_GROUP, false, 10, {{{DatatypeEnum::Buffer, true}}}, DeviceNode::DEFAULT_WAIT_FOR_MESSAGE}};

    /**
     * Output message of type MessageGroup
     */
    // Output out{*this, "out", Output::Type::MSender, {{DatatypeEnum::MessageGroup, false}}};
    DeviceNode::Output out{*this, {"out", DeviceNode::DEFAULT_GROUP, {{{DatatypeEnum::MessageGroup, false}}}}};

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

template class SyncBase<SyncTimestamp::Steady>;
template class SyncBase<SyncTimestamp::System>;

typedef SyncBase<SyncTimestamp::Steady> Sync;
typedef SyncBase<SyncTimestamp::System> SyncSystem;

}  // namespace node
}  // namespace dai
