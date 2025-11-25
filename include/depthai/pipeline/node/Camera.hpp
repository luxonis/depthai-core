#pragma once

// libraries
#include <optional>

#include "depthai/utility/spimpl.h"

// depthai
#include "depthai/capabilities/ImgFrameCapability.hpp"
#include "depthai/pipeline/DeviceNode.hpp"
#include "depthai/pipeline/datatype/CameraControl.hpp"
#include "depthai/properties/CameraProperties.hpp"

#ifdef DEPTHAI_HAVE_OPENCV_SUPPORT
    #include "depthai/pipeline/node/host/Replay.hpp"
#endif

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
                                std::optional<float> fps = std::nullopt,
                                std::optional<bool> enableUndistortion = std::nullopt);
    /**
     * Request output with advanced controls. Mainly to be used by custom node writers.
     */
    Node::Output* requestOutput(const Capability& capability, bool onHost) override;

    /**
     * Get a high resolution output with full FOV on the sensor.
     * By default the function will not use the resolutions higher than 5000x4000, as those often need a lot of resources,
     * making them hard to use in combination with other nodes.
     * @param type Type of the output (NV12, BGR, ...) - by default it's auto-selected for best performance
     * @param fps FPS of the output - by default it's auto-selected to highest possible that a sensor config support or 30, whichever is lower
     * @param useHighestResolution If true, the function will use the highest resolution available on the sensor, even if it's higher than 5000x4000
     */
    Node::Output* requestFullResolutionOutput(std::optional<ImgFrame::Type> type = std::nullopt,
                                              std::optional<float> fps = std::nullopt,
                                              bool useHighestResolution = false);

    /**
     * Request output with isp resolution. The fps does not vote.
     */
    Node::Output* requestIspOutput(std::optional<float> fps = std::nullopt);
    /**
     * Build with a specific board socket
     * @param boardSocket Board socket to use
     * @param sensorResolution Sensor resolution to use - by default it's auto-detected from the requested outputs
     * @param sensorFps Sensor FPS to use - by default it's auto-detected from the requested outputs (maximum is used)
     */
    std::shared_ptr<Camera> build(dai::CameraBoardSocket boardSocket = dai::CameraBoardSocket::AUTO,
                                  std::optional<std::pair<uint32_t, uint32_t>> sensorResolution = std::nullopt,
                                  std::optional<float> sensorFps = std::nullopt);

    /**
     * Set the sensor type to use
     * @param sensorType Sensor type to use
     */
    std::shared_ptr<Camera> setSensorType(CameraSensorType sensorType) {
        getProperties().sensorType = sensorType;
        return std::dynamic_pointer_cast<Camera>(shared_from_this());
    }

    /**
     * Get the sensor type
     * @return Sensor type
     */
    inline CameraSensorType getSensorType() {
        return getProperties().sensorType;
    }

#ifdef DEPTHAI_HAVE_OPENCV_SUPPORT
    /**
     * Build with a specific board socket and mock input
     */
    std::shared_ptr<Camera> build(dai::CameraBoardSocket boardSocket, ReplayVideo& replay);

    /**
     * Build with mock input
     */
    std::shared_ptr<Camera> build(ReplayVideo& replay);
#endif

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
     * Input for mocking 'isp' functionality on RVC2.
     * Default queue is blocking with size 8
     */

    Input mockIsp{*this, {"mockIsp", DEFAULT_GROUP, true, 8, {{{DatatypeEnum::ImgFrame, false}}}, DEFAULT_WAIT_FOR_MESSAGE}};

    /**
     * Outputs ImgFrame message that carries RAW10-packed (MIPI CSI-2 format) frame data.
     *
     * Captured directly from the camera sensor, and the source for the 'isp' output.
     */
    Output raw{*this, {"raw", DEFAULT_GROUP, {{{DatatypeEnum::ImgFrame, false}}}}};

    /**
     * Retrieves which board socket to use
     * @returns Board socket to use
     */
    CameraBoardSocket getBoardSocket() const;

    /**
     * Set number of frames in raw pool (will be automatically reduced if the maximum pool memory size is exceeded)
     * @param num Number of frames
     * @return Shared pointer to the camera node
     */
    std::shared_ptr<Camera> setRawNumFramesPool(int num);

    /**
     * Set maximum size of raw pool
     * @param size Maximum size in bytes of raw pool
     * @return Shared pointer to the camera node
     */
    std::shared_ptr<Camera> setMaxSizePoolRaw(int size);

    /**
     * Set number of frames in isp pool (will be automatically reduced if the maximum pool memory size is exceeded)
     * @param num Number of frames
     * @return Shared pointer to the camera node
     */
    std::shared_ptr<Camera> setIspNumFramesPool(int num);

    /**
     * Set maximum size of isp pool
     * @param size Maximum size in bytes of isp pool
     * @return Shared pointer to the camera node
     */
    std::shared_ptr<Camera> setMaxSizePoolIsp(int size);

    /**
     * Set number of frames in all pools (will be automatically reduced if the maximum pool memory size is exceeded)
     * @param raw Number of frames in raw pool
     * @param isp Number of frames in isp pool
     * @param outputs Number of frames in outputs pools
     * @return Shared pointer to the camera node
     */
    std::shared_ptr<Camera> setNumFramesPools(int raw, int isp, int outputs);

    /**
     * Set maximum memory size of all pools
     * @param raw Maximum size in bytes of raw pool
     * @param isp Maximum size in bytes of isp pool
     * @param outputs Maximum size in bytes of outputs pools
     * @return Shared pointer to the camera node
     */
    std::shared_ptr<Camera> setMaxSizePools(int raw, int isp, int outputs);

    /**
     * Set number of frames in pools for all outputs
     * @param num Number of frames in pools for all outputs
     * @return Shared pointer to the camera node
     */
    std::shared_ptr<Camera> setOutputsNumFramesPool(int num);

    /**
     * Set maximum size of pools for all outputs
     * @param size Maximum size in bytes of pools for all outputs
     * @return Shared pointer to the camera node
     */
    std::shared_ptr<Camera> setOutputsMaxSizePool(int size);

    /**
     * Get number of frames in raw pool
     * @return Number of frames
     */
    int getRawNumFramesPool() const;

    /**
     * Get maximum size of raw pool
     * @return Maximum size in bytes of raw pool
     */
    int getMaxSizePoolRaw() const;

    /**
     * Get number of frames in isp pool
     * @return Number of frames
     */
    int getIspNumFramesPool() const;

    /**
     * Get maximum size of isp pool
     * @return Maximum size in bytes of isp pool
     */
    int getMaxSizePoolIsp() const;

    /**
     * Get number of frames in outputs pool for all outputs
     * @return Number of frames
     */
    std::optional<int> getOutputsNumFramesPool() const;

    /**
     * Get maximum size of outputs pool for all outputs
     * @return Maximum size in bytes of image manip pool
     */
    std::optional<size_t> getOutputsMaxSizePool() const;

#ifdef DEPTHAI_HAVE_OPENCV_SUPPORT
    /**
     * Set mock ISP for Camera node. Automatically sets mockIsp size.
     * @param replay ReplayVideo node to use as mock ISP
     */
    Camera& setMockIsp(ReplayVideo& replay);
#endif

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

    float getMaxRequestedFps() const;
    uint32_t getMaxRequestedWidth() const;
    uint32_t getMaxRequestedHeight() const;

   protected:
    Properties& getProperties() override;
    bool isSourceNode() const override;
    NodeRecordParams getNodeRecordParams() const override;
    Input& getReplayInput() override;

   private:
    bool isBuilt = false;
    CameraFeatures cameraFeatures;
};

}  // namespace node
}  // namespace dai
