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
class MonoCamera : public NodeCRTP<Node, MonoCamera, MonoCameraProperties> {
   public:
    constexpr static const char* NAME = "MonoCamera";

   private:
    std::shared_ptr<RawCameraControl> rawControl;

   protected:
    Properties& getProperties();

   public:
    MonoCamera(const std::shared_ptr<PipelineImpl>& par, int64_t nodeId);
    MonoCamera(const std::shared_ptr<PipelineImpl>& par, int64_t nodeId, std::unique_ptr<Properties> props);

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
     * Outputs metadata-only ImgFrame message as an early indicator of an incoming frame.
     *
     * It's sent on the MIPI SoF (start-of-frame) event, just after the exposure of the current frame
     * has finished and before the exposure for next frame starts.
     * Could be used to synchronize various processes with camera capture.
     * Fields populated: camera id, sequence number, timestamp
     */
    Output frameEvent{*this, "frameEvent", Output::Type::MSender, {{DatatypeEnum::ImgFrame, false}}};

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

    /// Set number of frames in main (ISP output) pool
    void setNumFramesPool(int num);
    /// Set number of frames in raw pool
    void setRawNumFramesPool(int num);

    /// Get number of frames in main (ISP output) pool
    int getNumFramesPool() const;
    /// Get number of frames in raw pool
    int getRawNumFramesPool() const;
};

}  // namespace node
}  // namespace dai
