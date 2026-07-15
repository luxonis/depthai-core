#include "depthai/pipeline/node/Camera.hpp"

#include <stdexcept>

#include "depthai/pipeline/Pipeline.hpp"
#include "utility/RecordReplayImpl.hpp"

namespace dai::node {

std::shared_ptr<Camera> Camera::build(CameraBoardSocket boardSocket, ReplayVideo& replay) {
    auto cam = build(boardSocket);
    cam->setMockIsp(replay);
    return cam;
}

std::shared_ptr<Camera> Camera::build(ReplayVideo& replay) {
    auto cam = build(CameraBoardSocket::AUTO);
    cam->setMockIsp(replay);
    return cam;
}

Camera& Camera::setMockIsp(ReplayVideo& replay) {
    if(replay.getReplayVideoFile().empty()) {
        throw std::runtime_error("ReplayVideo video path not set");
    }

    auto [width, height] = replay.getSize();
    double fps = replay.getFps();
    if(width <= 0 || height <= 0) {
        const auto& [vidWidth, vidHeight, vidFps] = utility::getVideoSize(replay.getReplayVideoFile().string());
        width = vidWidth;
        height = vidHeight;
        fps = vidFps;
    }
    properties.mockIspWidth = width;
    properties.mockIspHeight = height;
    properties.mockIspFps = fps;

    auto device = getParentPipeline().getDefaultDevice();
    replay.setOutFrameType(device && device->getPlatform() == Platform::RVC2 ? ImgFrame::Type::YUV420p : ImgFrame::Type::NV12);
    replay.out.link(mockIsp);
    return *this;
}

}  // namespace dai::node
