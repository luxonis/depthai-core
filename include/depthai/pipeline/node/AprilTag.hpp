#pragma once

#include <depthai/pipeline/Node.hpp>

// standard
#include <fstream>

// shared
#include <depthai-shared/properties/AprilTagProperties.hpp>

#include "depthai/pipeline/datatype/AprilTagConfig.hpp"

namespace dai {
namespace node {

/**
 * @brief AprilTag node. Calculates spatial location data on a set of ROIs on depth map.
 */
class AprilTag : public Node {
   public:
    using Properties = dai::AprilTagProperties;

   private:
    std::string getName() const override;
    std::vector<Input> getInputs() override;
    std::vector<Output> getOutputs() override;
    nlohmann::json getProperties() override;
    std::shared_ptr<Node> clone() override;

    std::shared_ptr<RawAprilTagConfig> rawConfig;
    Properties properties;

   public:
    AprilTag(const std::shared_ptr<PipelineImpl>& par, int64_t nodeId);

    /**
     * Initial config to use when calculating spatial location data.
     */
    AprilTagConfig initialConfig;

    /**
     * Input AprilTagConfig message with ability to modify parameters in runtime.
     * Default queue is non-blocking with size 4.
     */
    Input inputConfig{*this, "inputConfig", Input::Type::SReceiver, false, 4, {{DatatypeEnum::AprilTagConfig, false}}};
    /**
     * Input message with depth data used to retrieve spatial information about detected object.
     * Default queue is non-blocking with size 4.
     */
    Input inputImage{*this, "inputImage", Input::Type::SReceiver, false, 4, {{DatatypeEnum::ImgFrame, false}}};

    /**
     * Outputs AprilTagData message that carries spatial location results.
     */
    Output outputImage{*this, "outputImage", Output::Type::MSender, {{DatatypeEnum::AprilTagData, false}}};

    /**
     * Passthrough message on which the calculation was performed.
     * Suitable for when input queue is set to non-blocking behavior.
     */
    Output passthroughInputImage{*this, "passthroughInputImage", Output::Type::MSender, {{DatatypeEnum::ImgFrame, false}}};

    // Functions to set properties
    /**
     * Specify whether or not wait until configuration message arrives to inputConfig Input.
     * @param wait True to wait for configuration message, false otherwise.
     */
    void setWaitForConfigInput(bool wait);
};

}  // namespace node
}  // namespace dai
