#pragma once

#include <depthai/pipeline/datatype/CameraControl.hpp>

#include "depthai/pipeline/Node.hpp"

// shared
#include <depthai-shared/properties/ColorCameraProperties.hpp>

namespace dai {
namespace node {
class ColorCamera : public Node {
    dai::ColorCameraProperties properties;

    std::string getName() const override;
    std::vector<Output> getOutputs() override;
    std::vector<Input> getInputs() override;
    nlohmann::json getProperties() override;
    std::shared_ptr<Node> clone() override;

    std::shared_ptr<RawCameraControl> rawControl;

   public:
    ColorCamera(const std::shared_ptr<PipelineImpl>& par, int64_t nodeId);

    CameraControl initialControl;

    /**
     * Input for ImageManipConfig message, which can modify crop paremeters in runtime
     * Default queue is non-blocking with size 8
     */
    Input inputConfig{*this, "inputConfig", Input::Type::SReceiver, false, 8, {{DatatypeEnum::ImageManipConfig, false}}};

    /**
     * Input for CameraControl message, which can modify camera parameters in runtime
     * Default queue is blocking with size 8
     */
    Input inputControl{*this, "inputControl", Input::Type::SReceiver, true, 8, {{DatatypeEnum::CameraControl, false}}};

    Output video{*this, "video", Output::Type::MSender, {{DatatypeEnum::ImgFrame, false}}};
    Output preview{*this, "preview", Output::Type::MSender, {{DatatypeEnum::ImgFrame, false}}};
    Output still{*this, "still", Output::Type::MSender, {{DatatypeEnum::ImgFrame, false}}};
    Output isp{*this, "isp", Output::Type::MSender, {{DatatypeEnum::ImgFrame, false}}};
    Output raw{*this, "raw", Output::Type::MSender, {{DatatypeEnum::ImgFrame, false}}};

    // Set which board socket to use
    void setBoardSocket(CameraBoardSocket boardSocket);

    // Get board socket
    CameraBoardSocket getBoardSocket() const;

    // Set which color camera to use
    [[deprecated("Use 'setBoardSocket()' instead")]] void setCamId(int64_t id);

    // Get which color camera to use
    [[deprecated("Use 'setBoardSocket()' instead")]] int64_t getCamId() const;

    // Set camera image orientation
    void setImageOrientation(CameraImageOrientation imageOrientation);

    // Get camera image orientation
    CameraImageOrientation getImageOrientation() const;

    // setColorOrder - RGB or BGR
    void setColorOrder(ColorCameraProperties::ColorOrder colorOrder);

    // getColorOrder - returns color order
    ColorCameraProperties::ColorOrder getColorOrder() const;

    // setInterleaved
    void setInterleaved(bool interleaved);

    // getInterleaved
    bool getInterleaved() const;

    // setFp16 mode (resultig output is FP16 0..255)
    void setFp16(bool fp16);

    // getFp16 mode (resultig output is FP16 0..255)
    bool getFp16() const;

    // set preview output size
    void setPreviewSize(int width, int height);

    // set 'video' output size
    void setVideoSize(int width, int height);

    // set 'still' output size
    void setStillSize(int width, int height);

    // set 'isp' output scaling
    void setIspScale(int numerator, int denominator);
    void setIspScaleFull(int horizNum, int horizDenom, int vertNum, int vertDenom);

    // set sensor resolution
    void setResolution(ColorCameraProperties::SensorResolution resolution);
    // get sensor resolution
    ColorCameraProperties::SensorResolution getResolution() const;

    void setFps(float fps);
    float getFps() const;

    // Returns preview size
    std::tuple<int, int> getPreviewSize() const;
    int getPreviewWidth() const;
    int getPreviewHeight() const;

    // Returns video size
    std::tuple<int, int> getVideoSize() const;
    int getVideoWidth() const;
    int getVideoHeight() const;

    // Returns still size
    std::tuple<int, int> getStillSize() const;
    int getStillWidth() const;
    int getStillHeight() const;

    // Returns sensor size
    std::tuple<int, int> getResolutionSize() const;
    int getResolutionWidth() const;
    int getResolutionHeight() const;

    void sensorCenterCrop();
    void setSensorCrop(float x, float y);

    std::tuple<float, float> getSensorCrop() const;
    float getSensorCropX() const;
    float getSensorCropY() const;

    // Node properties configuration
    void setWaitForConfigInput(bool wait);
    bool getWaitForConfigInput();

    void setPreviewKeepAspectRatio(bool keep);
    bool getPreviewKeepAspectRatio();

    void setEnablePreviewStillVideoStreams(bool enable);
};

}  // namespace node
}  // namespace dai
