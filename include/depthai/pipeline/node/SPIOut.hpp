#pragma once

#include <depthai/pipeline/Node.hpp>

// shared
#include <depthai-shared/properties/SPIOutProperties.hpp>

namespace dai {
namespace node {

/**
 * @brief SPIOut node. Sends messages over SPI.
 */
class SPIOut : public Node {
    dai::SPIOutProperties properties;

    std::string getName() const {
        return "SPIOut";
    }

    std::vector<Input> getInputs() {
        return {input};
    }

    std::vector<Output> getOutputs() {
        return {};
    }

    nlohmann::json getProperties() {
        nlohmann::json j;
        nlohmann::to_json(j, properties);
        return j;
    }

    std::shared_ptr<Node> clone() {
        return std::make_shared<SPIOut>(*this);
    }

   public:
    SPIOut(const std::shared_ptr<PipelineImpl>& par, int64_t nodeId) : Node(par, nodeId) {
        properties.busId = 0;
    }

    /**
     * Input for any type of messages to be transfered over SPI stream
     *
     * Default queue is blocking with size 8
     */
    Input input{*this, "in", Input::Type::SReceiver, true, 8, {{DatatypeEnum::Buffer, true}}};

    /**
     * Specifies stream name over which the node will send data
     *
     * @param name Stream name
     */
    void setStreamName(std::string name) {
        properties.streamName = name;
    }

    /**
     * Specifies SPI Bus number to use
     * @param id SPI Bus id
     */
    void setBusId(int id) {
        properties.busId = id;
    }
};

}  // namespace node
}  // namespace dai
