#pragma once
#include <optional>

#include "depthai/capabilities/ImgFrameCapability.hpp"
#include "depthai/common/CameraBoardSocket.hpp"
#include "depthai/common/CameraImageOrientation.hpp"
#include "depthai/pipeline/datatype/CameraControl.hpp"
#include "depthai/properties/Properties.hpp"

namespace dai {

/**
 *  Specify properties for ColorCamera such as camera ID, ...
 */
struct CameraProperties : PropertiesSerializable<Properties, CameraProperties> {
    static constexpr int AUTO = -1;

    std::vector<ImgFrameCapability> outputRequests;

    /**
     * Initial controls applied to ColorCamera node
     */
    CameraControl initialControl;

    /**
     * Which socket will color camera use
     */
    CameraBoardSocket boardSocket = CameraBoardSocket::AUTO;

    /**
     * Which camera name will color camera use
     */
    std::string cameraName = "";

    /**
     * Camera sensor image orientation / pixel readout
     */
    CameraImageOrientation imageOrientation = CameraImageOrientation::AUTO;

    /**
     * Select the camera sensor width
     */
    int32_t resolutionWidth = AUTO;
    /**
     * Select the camera sensor height
     */
    int32_t resolutionHeight = AUTO;

    /**
     * Select the mock isp width. Overrides resolutionWidth/height if mockIsp is connected.
     */
    int32_t mockIspWidth = AUTO;

    /**
     * Select the mock isp height. Overrides resolutionWidth/height if mockIsp is connected.
     */
    int32_t mockIspHeight = AUTO;

    /**
     * Select the mock isp fps. Overrides fps if mockIsp is connected.
     */
    float mockIspFps = AUTO;

    /**
     * Camera sensor FPS
     */
    float fps = AUTO;

    /**
     * Isp 3A rate (auto focus, auto exposure, auto white balance, camera controls etc.).
     * Default (0) matches the camera FPS, meaning that 3A is running on each frame.
     * Reducing the rate of 3A reduces the CPU usage on CSS, but also increases the convergence rate of 3A.
     * Note that camera controls will be processed at this rate. E.g. if camera is running at 30 fps, and camera control is sent at every frame,
     * but 3A fps is set to 15, the camera control messages will be processed at 15 fps rate, which will lead to queueing.

     */
    int isp3aFps = 0;

    /**
     * Pool sizes
     */
    int numFramesPoolRaw = 3;
    int numFramesPoolIsp = 3;
    int numFramesPoolVideo = 4;
    int numFramesPoolPreview = 4;
    int numFramesPoolStill = 4;
};

DEPTHAI_SERIALIZE_EXT(CameraProperties,
                      initialControl,
                      boardSocket,
                      cameraName,
                      imageOrientation,
                      resolutionWidth,
                      resolutionHeight,
                      mockIspWidth,
                      mockIspHeight,
                      fps,
                      isp3aFps,
                      numFramesPoolRaw,
                      numFramesPoolIsp,
                      numFramesPoolVideo,
                      numFramesPoolPreview,
                      numFramesPoolStill,
                      outputRequests);

}  // namespace dai
