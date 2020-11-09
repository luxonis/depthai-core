#include "depthai/pipeline/node/MonoCamera.hpp"

namespace dai {
namespace node {

    MonoCamera::MonoCamera(const std::shared_ptr<PipelineImpl>& par, int64_t nodeId) : Node(par, nodeId) {
        properties.camId = 1;
        properties.resolution = MonoCameraProperties::SensorResolution::THE_720_P;
        properties.fps = 30.0;
    }

    std::string MonoCamera::getName() {
        return "MonoCamera";
    }

    std::vector<Node::Output> MonoCamera::getOutputs() {
        return {out};
    }

    std::vector<Node::Input> MonoCamera::getInputs() {
        return {};
    }

    nlohmann::json MonoCamera::getProperties() {
        nlohmann::json j;
        nlohmann::to_json(j, properties);
        return j;
    }

    std::shared_ptr<Node> MonoCamera::clone() {
        return std::make_shared<std::decay<decltype(*this)>::type>(*this);
    }

    // Set which mono camera to use
    void MonoCamera::setCamId(int64_t id) {
        properties.camId = id;
    }

    // Get which mono camera to use
    int64_t MonoCamera::getCamId() const {
        return properties.camId;
    }

    void MonoCamera::setResolution(MonoCameraProperties::SensorResolution resolution) {
        properties.resolution = resolution;
    }

    void MonoCamera::setFps(float fps) {
        properties.fps = fps;
    }

}  // namespace node
}  // namespace dai
