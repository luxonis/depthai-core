#pragma once

#include <depthai/pipeline/datatype/CameraControl.hpp>

#include "depthai/common/CameraBoardSocket.hpp"
#include "depthai/pipeline/Node.hpp"

// shared
#include <depthai-shared/properties/MonoCameraProperties.hpp>

namespace dai {
namespace node {

/**
 * @brief MonoCamera node. For use with grayscale sensors.
 */
class MonoCamera : public Node {
   public:
    using Properties = dai::MonoCameraProperties;

   private:
    Properties properties;

    nlohmann::json getProperties() override;
    std::shared_ptr<Node> clone() override;

    std::shared_ptr<RawCameraControl> rawControl;

   public:
    std::string getName() const override;

    MonoCamera(const std::shared_ptr<PipelineImpl>& par, int64_t nodeId);

    /**
     * Initial control options to apply to sensor
     */
    CameraControl initialControl;

    /**
     * Input for CameraControl message, which can modify camera parameters in runtime
     * Default queue is blocking with size 8
     */
    Input inputControl{*this, "inputControl", Input::Type::SReceiver, true, 8, {{DatatypeEnum::CameraControl, false}}};

    /**
     * Outputs ImgFrame message that carries RAW8 encoded (grayscale) frame data.
     *
     * Suitable for use StereoDepth node. Processed by ISP
     */
    Output out{*this, "out", Output::Type::MSender, {{DatatypeEnum::ImgFrame, false}}};

    /**
     * Outputs ImgFrame message that carries RAW10-packed (MIPI CSI-2 format) frame data.
     *
     * Captured directly from the camera sensor
     */
    Output raw{*this, "raw", Output::Type::MSender, {{DatatypeEnum::ImgFrame, false}}};

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

    // Set which mono camera to use
    [[deprecated("Use 'setBoardSocket()' instead.")]] void setCamId(int64_t id);

    // Get which mono camera to use
    [[deprecated("Use 'getBoardSocket()' instead.")]] int64_t getCamId() const;

    /// Set camera image orientation
    void setImageOrientation(CameraImageOrientation imageOrientation);

    /// Get camera image orientation
    CameraImageOrientation getImageOrientation() const;

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

    /// Get sensor resolution as size
    std::tuple<int, int> getResolutionSize() const;
    /// Get sensor resolution width
    int getResolutionWidth() const;
    /// Get sensor resolution height
    int getResolutionHeight() const;
};

}  // namespace node
}  // namespace dai
