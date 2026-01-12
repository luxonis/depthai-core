#pragma once
#include <optional>

#include "depthai/capabilities/ImgFrameCapability.hpp"
#include "depthai/common/CameraBoardSocket.hpp"
#include "depthai/common/CameraImageOrientation.hpp"
#include "depthai/common/CameraSensorType.hpp"
#include "depthai/pipeline/datatype/CameraControl.hpp"
#include "depthai/properties/Properties.hpp"

namespace dai {

/**
 *  Specify properties for Camera such as camera ID, ...
 */
struct CameraProperties : PropertiesSerializable<Properties, CameraProperties> {
    static constexpr int AUTO = -1;

    std::vector<ImgFrameCapability> outputRequests;

    /**
     * Initial controls applied to the camera node
     */
    CameraControl initialControl;

    /**
     * Which socket will this camera node use
     */
    CameraBoardSocket boardSocket = CameraBoardSocket::AUTO;

    /**
     * Camera sensor type (you can pick one in case the given sensor supports multiple types)
     */
    CameraSensorType sensorType = CameraSensorType::AUTO;

    /**
     * Which camera name will this camera node use (e.g. "IMX378", "OV9282")
     */
    std::string cameraName = "";

    /**
     * Camera sensor image orientation / pixel readout
     */
    CameraImageOrientation imageOrientation = CameraImageOrientation::AUTO;

    /**
     * Select the camera sensor width (e.g. 1920, 1280, 640, etc.)
     */
    int32_t resolutionWidth = AUTO;
    /**
     * Select the camera sensor height (e.g. 1080, 720, 400, etc.)
     */
    int32_t resolutionHeight = AUTO;

    /**
     * Select the mock isp width. Overrides resolutionWidth/height if mockIsp is connected (e.g. 1920, 1280, 640, etc.)
     */
    int32_t mockIspWidth = AUTO;

    /**
     * Select the mock isp height. Overrides resolutionWidth/height if mockIsp is connected (e.g. 1080, 720, 400, etc.)
     */
    int32_t mockIspHeight = AUTO;

    /**
     * Select the mock isp fps. Overrides fps if mockIsp is connected (e.g. 30, 25, 20, etc.)
     */
    float mockIspFps = AUTO;

    /**
     * Camera sensor FPS (e.g. 30, 25, 20, etc.)
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
     * Number of frames in different pools and the maximum size in bytes of each pool (number of frames will be automatically reduced if the size is exceeded)
     */

    /** Raw pool */
    int numFramesPoolRaw = 3;
    int maxSizePoolRaw = 1024 * 1024 * 10;  // 10MB

    /** Isp pool */
    int numFramesPoolIsp = 3;
    int maxSizePoolIsp = 1024 * 1024 * 10;  // 10MB

    /** Video pool */
    int numFramesPoolVideo = 4;  // No max size in bytes for video pool, used in mono and color cameras only, those are deprecated

    /** Preview pool */
    int numFramesPoolPreview = 4;  // No max size in bytes for preview pool, used in mono and color cameras only, those are deprecated

    /** Still pool */
    int numFramesPoolStill = 4;  // No max size in bytes for still pool, used in mono and color cameras only, those are deprecated

    /** Outputs frame pools */
    std::optional<int> numFramesPoolOutputs;
    std::optional<int> maxSizePoolOutputs;

    ~CameraProperties() override;
};

DEPTHAI_SERIALIZE_EXT(CameraProperties,
                      initialControl,
                      boardSocket,
                      sensorType,
                      cameraName,
                      imageOrientation,
                      resolutionWidth,
                      resolutionHeight,
                      mockIspWidth,
                      mockIspHeight,
                      mockIspFps,
                      fps,
                      isp3aFps,
                      numFramesPoolRaw,
                      maxSizePoolRaw,
                      numFramesPoolIsp,
                      maxSizePoolIsp,
                      numFramesPoolVideo,
                      numFramesPoolPreview,
                      numFramesPoolStill,
                      numFramesPoolOutputs,
                      maxSizePoolOutputs,
                      outputRequests);

}  // namespace dai
