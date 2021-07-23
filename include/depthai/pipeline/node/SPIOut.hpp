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
   public:
    using Properties = dai::SPIOutProperties;

   private:
    Properties properties;

    nlohmann::json getProperties() override {
        nlohmann::json j;
        nlohmann::to_json(j, properties);
        return j;
    }

    std::shared_ptr<Node> clone() override {
        return std::make_shared<std::decay<decltype(*this)>::type>(*this);
    }

   public:
    std::string getName() const override {
        return "SPIOut";
    }

    SPIOut(const std::shared_ptr<PipelineImpl>& par, int64_t nodeId) : Node(par, nodeId) {
        properties.busId = 0;

        inputs = {&input};
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
