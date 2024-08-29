#pragma once

// libraries
#include <spimpl.h>

#include <optional>

// depthai
#include "depthai/capabilities/ImgFrameCapability.hpp"
#include "depthai/pipeline/DeviceNode.hpp"
#include "depthai/pipeline/datatype/CameraControl.hpp"
#include "depthai/properties/CameraProperties.hpp"
#include "depthai/utility/span.hpp"

namespace dai {
namespace node {

class Camera : public DeviceNodeCRTP<DeviceNode, Camera, CameraProperties>, public SourceNode {
   public:
    /**
     * Get video output with specified size.
     */
    Node::Output* requestOutput(std::pair<uint32_t, uint32_t> size,
                                std::optional<ImgFrame::Type> type = std::nullopt,
                                ImgResizeMode resizeMode = ImgResizeMode::CROP,
                                float fps = 30);
    /**
     * Request output with advanced controls. Mainly to be used by custom node writers.
     */
    Node::Output* requestOutput(const Capability& capability, bool onHost) override;

    /**
     * Get full resolution output
     */
    Node::Output* requestFullResolutionOutput(ImgFrame::Type type = ImgFrame::Type::NV12, float fps = 30);

    /**
     * Build with a specific board socket
     */
    std::shared_ptr<Camera> build(dai::CameraBoardSocket boardSocket = dai::CameraBoardSocket::AUTO);

    /**
     * Get max width of the camera (can only be called after build)
     */
    uint32_t getMaxWidth() const;

    /**
     * Get max height of the camera (can only be called after build)
     */
    uint32_t getMaxHeight() const;

    /**
     * Initial control options to apply to sensor
     */
    CameraControl initialControl;

    /**
     * Input for CameraControl message, which can modify camera parameters in runtime
     */
    Input inputControl{
        *this, {"inputControl", DEFAULT_GROUP, DEFAULT_BLOCKING, DEFAULT_QUEUE_SIZE, {{{DatatypeEnum::CameraControl, false}}}, DEFAULT_WAIT_FOR_MESSAGE}};

    /**
     * Retrieves which board socket to use
     * @returns Board socket to use
     */
    CameraBoardSocket getBoardSocket() const;

   private:
    class Impl;
    spimpl::impl_ptr<Impl> pimpl;

   public:  // internal usage
    constexpr static const char* NAME = "Camera";
    using DeviceNodeCRTP::DeviceNodeCRTP;
    [[nodiscard]] static std::shared_ptr<Camera> create() {
        auto node = std::make_shared<Camera>();
        node->buildInternal();
        return node;
    }
    [[nodiscard]] static std::shared_ptr<Camera> create(std::shared_ptr<Device>& defaultDevice) {
        auto node = std::make_shared<Camera>(defaultDevice);
        node->buildInternal();
        return node;
    }
    OutputMap dynamicOutputs{*this, "dynamicOutputs", {"", "", {{DatatypeEnum::ImgFrame, false}}}};
    Camera();
    explicit Camera(std::shared_ptr<Device>& defaultDevice);
    explicit Camera(std::unique_ptr<Properties> props);

    void buildStage1() override;

    std::pair<size_t, size_t> getMaxRequestedSize() const;
    float getMaxRequestedFps() const;

   protected:
    Properties& getProperties() override;
    bool isSourceNode() const override;
    NodeRecordParams getNodeRecordParams() const override;

   private:
    bool isBuilt = false;
    uint32_t maxWidth = 0;
    uint32_t maxHeight = 0;
    /*
    Output& getRecordOutput() override;
    Input& getReplayInput() override;
    */
};

}  // namespace node
}  // namespace dai
