#pragma once

#include <depthai/pipeline/DeviceNode.hpp>

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

    /**
     * Outputs SystemInformation[RVC4] message that carries various system information
     * like memory and CPU usage, temperatures, ...
     * For series 2 devices output SystemInformation message,
     * for series 3 and 4 devices output SystemInformationRVC4 message
     */
    Output out{*this, {"out", DEFAULT_GROUP, {{{DatatypeEnum::SystemInformation, false}, {DatatypeEnum::SystemInformationRVC4, false}}}}};

    /**
     * Specify logging rate, at which messages will be sent out
     * @param hz Sending rate in hertz (messages per second)
     */
    void setRate(float hz);

    /**
     * Gets logging rate, at which messages will be sent out
     */
    float getRate();

    void buildInternal() override;
};

}  // namespace node
}  // namespace dai
