#include "depthai/pipeline/node/ColorCamera.hpp"

namespace dai {
namespace node {

    ColorCamera::ColorCamera(const std::shared_ptr<PipelineImpl>& par, int64_t nodeId) : Node(par, nodeId) {
        properties.camId = 0;
        properties.colorOrder = ColorCameraProperties::ColorOrder::BGR;
        properties.interleaved = true;
        properties.previewHeight = 300;
        properties.previewWidth = 300;
        properties.resolution = ColorCameraProperties::SensorResolution::THE_1080_P;
        properties.fps = 30.0;
    }

    std::string ColorCamera::getName() {
        return "ColorCamera";
    }

    std::vector<Node::Output> ColorCamera::getOutputs() {
        return {video, preview, still};
    }

    std::vector<Node::Input> ColorCamera::getInputs() {
        return {};
    }

    nlohmann::json ColorCamera::getProperties() {
        nlohmann::json j;
        nlohmann::to_json(j, properties);
        return j;
    }

    std::shared_ptr<Node> ColorCamera::clone() {
        return std::make_shared<std::decay<decltype(*this)>::type>(*this);
    }

    // Set which color camera to use
    void ColorCamera::setCamId(int64_t id) {
        properties.camId = id;
    }
    // Get which color camera to use
    int64_t ColorCamera::getCamId() const {
        return properties.camId;
    }

    // setColorOrder - RGB or BGR
    void ColorCamera::setColorOrder(ColorCameraProperties::ColorOrder colorOrder) {
        properties.colorOrder = colorOrder;
    }

    // getColorOrder - returns color order
    ColorCameraProperties::ColorOrder ColorCamera::getColorOrder() const {
        return properties.colorOrder;
    }

    // setInterleaved
    void ColorCamera::setInterleaved(bool interleaved) {
        properties.interleaved = interleaved;
    }

    // set preview output size
    void ColorCamera::setPreviewSize(int width, int height) {
        properties.previewWidth = width;
        properties.previewHeight = height;
    }

    void ColorCamera::setResolution(ColorCameraProperties::SensorResolution resolution) {
        properties.resolution = resolution;
    }

    void ColorCamera::setFps(float fps) {
        properties.fps = fps;
    }

}  // namespace node
}  // namespace dai
