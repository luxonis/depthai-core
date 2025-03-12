#pragma once
#include <optional>

#include "depthai/capabilities/ImgFrameCapability.hpp"
#include "depthai/common/CameraBoardSocket.hpp"
#include "depthai/common/CameraImageOrientation.hpp"
#include "depthai/common/CameraSensorType.hpp"
#include "depthai/common/optional.hpp"
#include "depthai/common/variant.hpp"
#include "depthai/pipeline/datatype/CameraControl.hpp"
#include "depthai/properties/Properties.hpp"

namespace dai {

/**
 *  Specify properties for ColorCamera such as camera ID, ...
 */
struct CameraProperties : PropertiesSerializable<Properties, CameraProperties> {
    static constexpr int AUTO = -1;

    struct IspScale {
        int32_t horizNumerator = 0;
        int32_t horizDenominator = 0;
        int32_t vertNumerator = 0;
        int32_t vertDenominator = 0;

        DEPTHAI_SERIALIZE(IspScale, horizNumerator, horizDenominator, vertNumerator, vertDenominator);
    };

    /**
     * For 24 bit color these can be either RGB or BGR
     */
    enum class ColorOrder : int32_t { BGR, RGB };

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
     * For 24 bit color these can be either RGB or BGR
     */
    ColorOrder colorOrder = ColorOrder::BGR;
    /**
     * Are colors interleaved (R1G1B1, R2G2B2, ...) or planar (R1R2..., G1G2..., B1B2)
     */
    bool interleaved = true;
    /**
     * Are values FP16 type (0.0 - 255.0)
     */
    bool fp16 = false;

    static constexpr uint32_t DEFAULT_PREVIEW_HEIGHT = 300;
    static constexpr uint32_t DEFAULT_PREVIEW_WIDTH = 300;

    /**
     * Preview frame output height
     */
    uint32_t previewHeight = DEFAULT_PREVIEW_HEIGHT;
    /**
     * Preview frame output width
     */
    uint32_t previewWidth = DEFAULT_PREVIEW_WIDTH;

    /**
     * Preview frame output width
     */
    int32_t videoWidth = AUTO;

    /**
     * Preview frame output height
     */
    int32_t videoHeight = AUTO;

    /**
     * Preview frame output width
     */
    int32_t stillWidth = AUTO;

    /**
     * Preview frame output height
     */
    int32_t stillHeight = AUTO;

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
     * Initial sensor crop, -1 signifies center crop
     */
    float sensorCropX = AUTO;
    float sensorCropY = AUTO;

    /**
     * Whether to keep aspect ratio of input (video/preview size) or not
     */
    bool previewKeepAspectRatio = false;

    /**
     * Configure scaling for `isp` output.
     */
    IspScale ispScale;

    /// Type of sensor, specifies what kind of postprocessing is performed
    CameraSensorType sensorType = CameraSensorType::AUTO;

    /**
     * Pool sizes
     */
    int numFramesPoolRaw = 3;
    int numFramesPoolIsp = 3;
    int numFramesPoolVideo = 4;
    int numFramesPoolPreview = 4;
    int numFramesPoolStill = 4;

    /**
     * Warp mesh source
     */
    enum class WarpMeshSource { AUTO = -1, NONE, CALIBRATION, URI };
    WarpMeshSource warpMeshSource = WarpMeshSource::AUTO;
    std::string warpMeshUri = "";
    int warpMeshWidth, warpMeshHeight;
    /**
     * Free scaling parameter between 0 (when all the pixels in the undistorted image are valid)
     * and 1 (when all the source image pixels are retained in the undistorted image).
     * On some high distortion lenses, and/or due to rectification (image rotated) invalid areas may appear even with alpha=0,
     * in these cases alpha < 0.0 helps removing invalid areas.
     * See getOptimalNewCameraMatrix from opencv for more details.
     */
    std::optional<float> calibAlpha;
    int warpMeshStepWidth = 32;
    int warpMeshStepHeight = 32;

    /**
     * Configures whether the camera `raw` frames are saved as MIPI-packed to memory.
     * The packed format is more efficient, consuming less memory on device, and less data
     * to send to host: RAW10: 4 pixels saved on 5 bytes, RAW12: 2 pixels saved on 3 bytes.
     * When packing is disabled (`false`), data is saved lsb-aligned, e.g. a RAW10 pixel
     * will be stored as uint16, on bits 9..0: 0b0000'00pp'pppp'pppp.
     * Default is auto: enabled for standard color/monochrome cameras where ISP can work
     * with both packed/unpacked, but disabled for other cameras like ToF.
     */
    std::optional<bool> rawPacked;
};

DEPTHAI_SERIALIZE_EXT(CameraProperties,
                      initialControl,
                      boardSocket,
                      cameraName,
                      imageOrientation,
                      colorOrder,
                      interleaved,
                      fp16,
                      previewHeight,
                      previewWidth,
                      videoWidth,
                      videoHeight,
                      stillWidth,
                      stillHeight,
                      resolutionWidth,
                      resolutionHeight,
                      mockIspWidth,
                      mockIspHeight,
                      fps,
                      isp3aFps,
                      sensorCropX,
                      sensorCropY,
                      previewKeepAspectRatio,
                      ispScale,
                      sensorType,
                      numFramesPoolRaw,
                      numFramesPoolIsp,
                      numFramesPoolVideo,
                      numFramesPoolPreview,
                      numFramesPoolStill,
                      warpMeshSource,
                      warpMeshUri,
                      warpMeshWidth,
                      warpMeshHeight,
                      calibAlpha,
                      warpMeshStepWidth,
                      warpMeshStepHeight,
                      rawPacked,
                      outputRequests);

}  // namespace dai
