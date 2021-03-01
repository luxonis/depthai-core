#pragma once

#include <depthai/pipeline/datatype/CameraControl.hpp>

#include "depthai/pipeline/Node.hpp"

// shared
#include <depthai-shared/properties/ColorCameraProperties.hpp>

namespace dai {
namespace node {

/**
 * @brief ColorCamera node. For use with color sensors.
 */
class ColorCamera : public Node {
    using Properties = dai::ColorCameraProperties;
    Properties properties;

    std::string getName() const override;
    std::vector<Output> getOutputs() override;
    std::vector<Input> getInputs() override;
    nlohmann::json getProperties() override;
    std::shared_ptr<Node> clone() override;

    std::shared_ptr<RawCameraControl> rawControl;

   public:
    /**
     * Constructs ColorCamera node.
     */
    ColorCamera(const std::shared_ptr<PipelineImpl>& par, int64_t nodeId);

    /**
     * Initial control options to apply to sensor
     */
    CameraControl initialControl;

    /**
     * Input for ImageManipConfig message, which can modify crop paremeters in runtime
     *
     * Default queue is non-blocking with size 8
     */
    Input inputConfig{*this, "inputConfig", Input::Type::SReceiver, false, 8, {{DatatypeEnum::ImageManipConfig, false}}};

    /**
     * Input for CameraControl message, which can modify camera parameters in runtime
     *
     * Default queue is blocking with size 8
     */
    Input inputControl{*this, "inputControl", Input::Type::SReceiver, true, 8, {{DatatypeEnum::CameraControl, false}}};

    /**
     * Outputs ImgFrame message that carries NV12 encoded (YUV420, UV plane interleaved) frame data.
     *
     * Suitable for use with VideoEncoder node
     */
    Output video{*this, "video", Output::Type::MSender, {{DatatypeEnum::ImgFrame, false}}};

    /**
     * Outputs ImgFrame message that carries BGR/RGB planar/interleaved encoded frame data.
     *
     * Suitable for use with NeuralNetwork node
     */
    Output preview{*this, "preview", Output::Type::MSender, {{DatatypeEnum::ImgFrame, false}}};

    /**
     * Outputs ImgFrame message that carries NV12 encoded (YUV420, UV plane interleaved) frame data.
     *
     * The message is sent only when a CameraControl message arrives to inputControl with captureStill command set.
     */
    Output still{*this, "still", Output::Type::MSender, {{DatatypeEnum::ImgFrame, false}}};

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

    /// Set which color camera to use
    [[deprecated("Use 'setBoardSocket()' instead")]] void setCamId(int64_t id);

    /// Get which color camera to use
    [[deprecated("Use 'setBoardSocket()' instead")]] int64_t getCamId() const;

    /// Set camera image orientation
    void setImageOrientation(CameraImageOrientation imageOrientation);

    /// Get camera image orientation
    CameraImageOrientation getImageOrientation() const;

    /// Set color order of preview output images. RGB or BGR
    void setColorOrder(ColorCameraProperties::ColorOrder colorOrder);

    /// Get color order of preview output frames. RGB or BGR
    ColorCameraProperties::ColorOrder getColorOrder() const;

    /// Set planar or interleaved data of preview output frames
    void setInterleaved(bool interleaved);

    /// Get planar or interleaved data of preview output frames
    bool getInterleaved() const;

    /// Set fp16 (0..255) data type of preview output frames
    void setFp16(bool fp16);

    /// Get fp16 (0..255) data of preview output frames
    bool getFp16() const;

    /// Set preview output size
    void setPreviewSize(int width, int height);

    /// Set video output size
    void setVideoSize(int width, int height);

    /// Set still output size
    void setStillSize(int width, int height);

    /// Set sensor resolution
    void setResolution(Properties::SensorResolution resolution);

    /// Get sensor resolution
    Properties::SensorResolution getResolution() const;

    /**
     * Set rate at which camera should produce frames
     * @param fps Rate in frames per second
     */
    void setFps(float fps);

    /**
     * Get rate at which camera should produce frames
     * @returns Rate in frames per second
     */
    float getFps() const;

    /// Get preview size as tuple
    std::tuple<int, int> getPreviewSize() const;
    /// Get preview width
    int getPreviewWidth() const;
    /// Get preview height
    int getPreviewHeight() const;

    /// Get video size as tuple
    std::tuple<int, int> getVideoSize() const;
    /// Get video width
    int getVideoWidth() const;
    /// Get video height
    int getVideoHeight() const;

    /// Get still size as tuple
    std::tuple<int, int> getStillSize() const;
    /// Get still width
    int getStillWidth() const;
    /// Get still height
    int getStillHeight() const;

    /// Get sensor resolution as size
    std::tuple<int, int> getResolutionSize() const;
    /// Get sensor resolution width
    int getResolutionWidth() const;
    /// Get sensor resolution height
    int getResolutionHeight() const;

    /**
     * Specify sensor center crop.
     * Resolution size / video size
     */
    void sensorCenterCrop();

    /**
     * Specifies sensor crop rectangle
     * @param x Top left X coordinate
     * @param y Top left Y coordinate
     */
    void setSensorCrop(float x, float y);

    /**
     * @returns Sensor top left crop coordinates
     */
    std::tuple<float, float> getSensorCrop() const;
    /// Get sensor top left x crop coordinate
    float getSensorCropX() const;
    /// Get sensor top left y crop coordinate
    float getSensorCropY() const;

    // Node properties configuration
    /**
     * Specify to wait until inputConfig receives a configuration message,
     * before sending out a frame.
     * @param wait True to wait for inputConfig message, false otherwise
     */
    void setWaitForConfigInput(bool wait);

    /**
     * @see setWaitForConfigInput
     * @returns True if wait for inputConfig message, false otherwise
     */
    bool getWaitForConfigInput();

    /**
     * Specifies whether preview output should preserve aspect ratio,
     * after downscaling from video size or not.
     *
     * @param keep If true, a larger crop region will be considered to still be able to
     * create the final image in the specified aspect ratio. Otherwise video size is resized to fit preview size
     */
    void setPreviewKeepAspectRatio(bool keep);

    /**
     * @see setPreviewKeepAspectRatio
     * @returns Preview keep aspect ratio option
     */
    bool getPreviewKeepAspectRatio();
};

}  // namespace node
}  // namespace dai
