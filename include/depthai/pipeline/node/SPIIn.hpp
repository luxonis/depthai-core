#pragma once

#include <depthai/pipeline/Node.hpp>

// shared
#include <depthai-shared/properties/SPIInProperties.hpp>

namespace dai {
namespace node {

/**
 * @brief SPIIn node. Receives messages over SPI.
 */
class SPIIn : public Node {
   public:
    using Properties = dai::SPIInProperties;

   private:
    Properties properties;

    std::string getName() const override {
        return "SPIIn";
    }

    std::vector<Input> getInputs() override {
        return {};
    }

    std::vector<Output> getOutputs() override {
        return {out};
    }

    nlohmann::json getProperties() override {
        nlohmann::json j;
        nlohmann::to_json(j, properties);
        return j;
    }

    std::shared_ptr<Node> clone() override {
        return std::make_shared<std::decay<decltype(*this)>::type>(*this);
    }

   public:
    SPIIn(const std::shared_ptr<PipelineImpl>& par, int64_t nodeId) : Node(par, nodeId) {
        properties.busId = 0;
    }

    /**
     * Outputs message of same type as send from host.
     */
    Output out{*this, "out", Output::Type::MSender, {{DatatypeEnum::Buffer, true}}};

    /**
     * Specifies stream name over which the node will receive data
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
