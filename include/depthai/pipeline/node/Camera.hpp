#pragma once

// libraries
#include <spimpl.h>

// depthai
#include "depthai/capabilities/ImgFrameCapability.hpp"
#include "depthai/pipeline/DeviceNode.hpp"
#include "depthai/pipeline/datatype/CameraControl.hpp"
#include "depthai/properties/CameraProperties.hpp"
#include "depthai/utility/span.hpp"

namespace dai {
namespace node {

class Camera : public DeviceNodeCRTP<DeviceNode, Camera, CameraProperties> {
   public:
    /**
     * Get video output with specified size.
     */
    Node::Output* requestOutput(std::pair<uint32_t, uint32_t> size,
                                ImgFrame::Type type = ImgFrame::Type::NV12,
                                ImgResizeMode resizeMode = ImgResizeMode::CROP,
                                uint32_t fps = 30);
    /**
     * Request output with advanced controls. Mainly to be used by custom node writers.
     */
    Node::Output* requestOutput(const Capability& capability, bool onHost) override;

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
     * Specify which board socket to use
     * @param boardSocket Board socket to use
     */
    void setBoardSocket(CameraBoardSocket boardSocket);

    /**
     * Retrieves which board socket to use
     * @returns Board socket to use
     */
    CameraBoardSocket getBoardSocket() const;

    /**
     * Specify which camera to use by name
     * @param name Name of the camera to use
     */
    void setCamera(std::string name);

    /**
     * Retrieves which camera to use by name
     * @returns Name of the camera to use
     */
    std::string getCamera() const;

   private:
    class Impl;
    spimpl::impl_ptr<Impl> pimpl;

   public:  // internal usage
    constexpr static const char* NAME = "Camera";
    using DeviceNodeCRTP::DeviceNodeCRTP;
    [[nodiscard]] static std::shared_ptr<Camera> create() {
        auto node = std::make_shared<Camera>();
        node->build();
        return node;
    }
    [[nodiscard]] static std::shared_ptr<Camera> create(std::shared_ptr<Device>& defaultDevice) {
        auto node = std::make_shared<Camera>(defaultDevice);
        node->build();
        return node;
    }
    OutputMap dynamicOutputs{*this, "dynamicOutputs", {"", "", {{DatatypeEnum::ImgFrame, false}}}};
    Camera();
    explicit Camera(std::shared_ptr<Device>& defaultDevice);
    explicit Camera(std::unique_ptr<Properties> props);

    std::shared_ptr<Camera> build();
    void buildStage1() override;

   protected:
    Properties& getProperties() override;
    bool isSourceNode() const override;
    utility::NodeRecordParams getNodeRecordParams() const override;
    /*
    Output& getRecordOutput() override;
    Input& getReplayInput() override;
    */

    bool isBuild = false;
    bool needsBuild() override {
        return !isBuild;
    }
};

}  // namespace node
}  // namespace dai
