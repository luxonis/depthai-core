#pragma once

#include "depthai/pipeline/Node.hpp"

// shared
#include <depthai-shared/common/CameraBoardSocket.hpp>
#include <depthai-shared/properties/MonoCameraProperties.hpp>

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

    Input inputControl{*this, "inputControl", Input::Type::SReceiver, {{DatatypeEnum::CameraControl, false}}};

    Output out{*this, "out", Output::Type::MSender, {{DatatypeEnum::ImgFrame, false}}};

    // Set which board socket to use
    void setBoardSocket(CameraBoardSocket boardSocket);

    // Get board socket
    CameraBoardSocket getBoardSocket() const;

    // Set which mono camera to use
    [[deprecated("Use 'setBoardSocket()' instead.")]] void setCamId(int64_t id);

    // Get which mono camera to use
    [[deprecated("Use 'getBoardSocket()' instead.")]] int64_t getCamId() const;

    // Set camera image orientation
    void setImageOrientation(CameraImageOrientation imageOrientation);

    // Get camera image orientation
    CameraImageOrientation getImageOrientation() const;

    void setResolution(MonoCameraProperties::SensorResolution resolution);
    MonoCameraProperties::SensorResolution getResolution() const;

    void setFps(float fps);
    float getFps() const;

    std::tuple<int, int> getResolutionSize() const;
    int getResolutionWidth() const;
    int getResolutionHeight() const;
};

}  // namespace node
}  // namespace dai
