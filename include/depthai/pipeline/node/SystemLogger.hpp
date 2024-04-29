#pragma once

#include <depthai/pipeline/DeviceNode.hpp>
#include <depthai/pipeline/node/Pool.hpp>

// shared
#include <depthai/properties/SystemLoggerProperties.hpp>

namespace dai {
namespace node {

/**
 * @brief SystemLogger node. Send system information periodically.
 */
class SystemLogger : public DeviceNodeCRTP<DeviceNode, SystemLogger, SystemLoggerProperties> {
   public:
    constexpr static const char* NAME = "SystemLogger";
    using DeviceNodeCRTP::DeviceNodeCRTP;
    std::shared_ptr<SystemLogger> build();

    /**
     * Outputs SystemInformation[S3] message that carries various system information
     * like memory and CPU usage, temperatures, ...
     * For series 2 devices outputs SystemInformation message,
     * for series 3 devices outputs SystemInformationS3 message
     */
    Output out{*this, {.name = "out", .types = {{DatatypeEnum::SystemInformation, false}, {DatatypeEnum::SystemInformationS3, false}}}};

    /**
     * Optional - consumes an SystemInformation message from pool to send outwards
     * Otherwise uses dynamic allocation and/or default pool
     */

    // TODO(before mainline) - Clean up - add pools back in when ready
    // Input inputPool{
    //     true, *this, "inputPool", Input::Type::MReceiver, false, 4, {{DatatypeEnum::SystemInformation, false}, {DatatypeEnum::SystemInformationS3, false}}};

    /**
     * Default pool that is linked to inputPool input
     */
    // Subnode<Pool> pool{*this, "pool"};

    /**
     * Specify logging rate, at which messages will be sent out
     * @param hz Sending rate in hertz (messages per second)
     */
    void setRate(float hz);

    /**
     * Gets logging rate, at which messages will be sent out
     */
    float getRate();
};

}  // namespace node
}  // namespace dai
