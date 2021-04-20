#pragma once

#include "depthai/pipeline/Node.hpp"

// shared
#include <depthai-shared/properties/IMUProperties.hpp>

namespace dai {
namespace node {

/**
 * @brief IMU node. For use with color sensors.
 */
class IMU : public Node {
   public:
    using Properties = dai::IMUProperties;

   private:
    Properties properties;

    std::string getName() const override;
    std::vector<Output> getOutputs() override;
    std::vector<Input> getInputs() override;
    nlohmann::json getProperties() override;
    std::shared_ptr<Node> clone() override;

   public:
    /**
     * Constructs IMU node.
     */
    IMU(const std::shared_ptr<PipelineImpl>& par, int64_t nodeId);

    /**
     * Outputs ImgFrame message that carries BGR/RGB planar/interleaved encoded frame data.
     *
     * Suitable for use with NeuralNetwork node
     */
    Output out{*this, "out", Output::Type::MSender, {{DatatypeEnum::IMUDatas, false}}};
};

}  // namespace node
}  // namespace dai
