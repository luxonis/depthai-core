#pragma once

#include "depthai/pipeline/Node.hpp"

// shared
#include <depthai-shared/pb/properties/MonoCameraProperties.hpp>

namespace dai {
namespace node {
class MonoCamera : public Node {
    dai::MonoCameraProperties properties;

    std::string getName() const override;
    std::vector<Output> getOutputs() override;
    std::vector<Input> getInputs() override;
    nlohmann::json getProperties() override;
    std::shared_ptr<Node> clone() override;

   public:
    MonoCamera(const std::shared_ptr<PipelineImpl>& par, int64_t nodeId);

    Output out{*this, "out", Output::Type::MSender, {{DatatypeEnum::ImgFrame, false}}};

    // Set which mono camera to use
    void setCamId(int64_t id);

    // Get which mono camera to use
    int64_t getCamId() const;

    void setResolution(MonoCameraProperties::SensorResolution resolution);
    void setFps(float fps);
};

}  // namespace node
}  // namespace dai
