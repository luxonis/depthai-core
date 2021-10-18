#pragma once

#include <depthai/pipeline/Node.hpp>

// shared
#include <depthai-shared/properties/SystemLoggerProperties.hpp>

namespace dai {
namespace node {

/**
 * @brief SystemLogger node. Send system information periodically.
 */
class SystemLogger : public Node {
    using Properties = dai::SystemLoggerProperties;
    /// Underlying properties
    Properties& properties;

    Properties& getProperties() override;
    std::shared_ptr<Node> clone() override;

   public:
    std::string getName() const override;

    SystemLogger(const std::shared_ptr<PipelineImpl>& par, int64_t nodeId);
    SystemLogger(const std::shared_ptr<PipelineImpl>& par, int64_t nodeId, std::unique_ptr<Properties> props);

    /**
     * Outputs SystemInformation message that carries various system information
     * like memory and CPU usage, temperatures, ...
     */
    Output out{*this, "out", Output::Type::MSender, {{DatatypeEnum::SystemInformation, false}}};

    /**
     * Specify logging rate, at which messages will be sent to out output
     * @param hz Sending rate in hertz (messages per second)
     */
    void setRate(float hz);
};

}  // namespace node
}  // namespace dai
