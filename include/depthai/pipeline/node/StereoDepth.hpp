#pragma once

#include "depthai/pipeline/Node.hpp"

// shared
#include "depthai-shared/properties/StereoDepthProperties.hpp"
#include "depthai/pipeline/datatype/StereoDepthConfig.hpp"

namespace dai {
namespace node {

/**
 * @brief StereoDepth node. Compute stereo disparity and depth from left-right image pair.
 */
class StereoDepth : public Node {
   public:
    using Properties = dai::StereoDepthProperties;

   private:
    Properties properties;

    nlohmann::json getProperties() override;
    std::shared_ptr<Node> clone() override;
    std::shared_ptr<RawStereoDepthConfig> rawConfig;

   public:
    std::string getName() const override;

    StereoDepth(const std::shared_ptr<PipelineImpl>& par, int64_t nodeId);

    /**
     * Initial config to use for StereoDepth.
     */
    StereoDepthConfig initialConfig;

    /**
     * Input StereoDepthConfig message with ability to modify parameters in runtime.
     * Default queue is non-blocking with size 4.
     */
    Input inputConfig{*this, "inputConfig", Input::Type::SReceiver, false, 4, {{DatatypeEnum::StereoDepthConfig, false}}};

    /**
     * Input for left ImgFrame of left-right pair
     *
     * Default queue is non-blocking with size 8
     */
    Input left{*this, "left", Input::Type::SReceiver, false, 8, {{DatatypeEnum::ImgFrame, true}}};

    /**
     * Input for right ImgFrame of left-right pair
     *
     * Default queue is non-blocking with size 8
     */
    Input right{*this, "right", Input::Type::SReceiver, false, 8, {{DatatypeEnum::ImgFrame, true}}};

    /**
     * Outputs ImgFrame message that carries RAW16 encoded (0..65535) depth data in millimeters.
     *
     * Non-determined / invalid depth values are set to 0
     */
    Output depth{*this, "depth", Output::Type::MSender, {{DatatypeEnum::ImgFrame, false}}};

    /**
     * Outputs ImgFrame message that carries RAW8 / RAW16 encoded disparity data:
     * RAW8 encoded (0..95) for standard mode;
     * RAW8 encoded (0..190) for extended disparity mode;
     * RAW16 encoded (0..3040) for subpixel disparity mode (32 subpixel levels on top of standard mode).
     */
    Output disparity{*this, "disparity", Output::Type::MSender, {{DatatypeEnum::ImgFrame, false}}};

    /**
     * Passthrough ImgFrame message from 'left' Input.
     */
    Output syncedLeft{*this, "syncedLeft", Output::Type::MSender, {{DatatypeEnum::ImgFrame, false}}};

    /**
     * Passthrough ImgFrame message from 'right' Input.
     */
    Output syncedRight{*this, "syncedRight", Output::Type::MSender, {{DatatypeEnum::ImgFrame, false}}};

    /**
     * Outputs ImgFrame message that carries RAW8 encoded (grayscale) rectified frame data.
     */
    Output rectifiedLeft{*this, "rectifiedLeft", Output::Type::MSender, {{DatatypeEnum::ImgFrame, false}}};

    /**
     * Outputs ImgFrame message that carries RAW8 encoded (grayscale) rectified frame data.
     */
    Output rectifiedRight{*this, "rectifiedRight", Output::Type::MSender, {{DatatypeEnum::ImgFrame, false}}};

    /**
     * Specify local filesystem path to the calibration file
     * @param path Path to calibration file. If empty use EEPROM
     */
    [[deprecated("Use 'Pipeline::setCalibrationData()' instead")]] void loadCalibrationFile(const std::string& path);

    /**
     * Specify calibration data as a vector of bytes
     * @param path Calibration data. If empty use EEPROM
     */
    [[deprecated("Use 'Pipeline::setCalibrationData()' instead")]] void loadCalibrationData(const std::vector<std::uint8_t>& data);

    /**
     * Specify that a passthrough/dummy calibration should be used,
     * when input frames are already rectified (e.g. sourced from recordings on the host)
     */
    [[deprecated("Use 'Stereo::setRectification(false)' instead")]] void setEmptyCalibration();

    /**
     * Specify local filesystem paths to the mesh calibration files for 'left' and 'right' inputs.
     *
     * When a mesh calibration is set, it overrides the camera intrinsics/extrinsics matrices.
     * Mesh format: a sequence of (y,x) points as 'float' with coordinates from the input image
     * to be mapped in the output. The mesh can be subsampled, configured by `setMeshStep`.
     *
     * With a 1280x800 resolution and the default (16,16) step, the required mesh size is:
     *
     * width: 1280 / 16 + 1 = 81
     *
     * height: 800 / 16 + 1 = 51
     */
    void loadMeshFiles(const std::string& pathLeft, const std::string& pathRight);

    /**
     * Specify mesh calibration data for 'left' and 'right' inputs, as vectors of bytes.
     * See `loadMeshFiles` for the expected data format
     */
    void loadMeshData(const std::vector<std::uint8_t>& dataLeft, const std::vector<std::uint8_t>& dataRight);

    /**
     * Set the distance between mesh points. Default: (16, 16)
     */
    void setMeshStep(int width, int height);

    /**
     * Specify input resolution size
     *
     * Optional if MonoCamera exists, otherwise necessary
     */
    void setInputResolution(int width, int height);

    /**
     * Specify disparity/depth output resolution size, implemented by scaling.
     *
     * Currently only applicable when aligning to RGB camera
     */
    void setOutputSize(int width, int height);

    /**
     * Specifies whether the frames resized by `setOutputSize` should preserve aspect ratio,
     * with potential cropping when enabled. Default `true`
     */
    void setOutputKeepAspectRatio(bool keep);

    /**
     * @param median Set kernel size for disparity/depth median filtering, or disable
     */
    [[deprecated("Use 'initialConfig.setMedianFilter()' instead")]] void setMedianFilter(dai::MedianFilter median);

    /**
     * @param align Set the disparity/depth alignment: centered (between the 'left' and 'right' inputs),
     * or from the perspective of a rectified output stream
     */
    void setDepthAlign(Properties::DepthAlign align);

    /**
     * @param camera Set the camera from whose perspective the disparity/depth will be aligned
     */
    void setDepthAlign(CameraBoardSocket camera);

    /**
     * Confidence threshold for disparity calculation
     * @param confThr Confidence threshold value 0..255
     */
    [[deprecated("Use 'initialConfig.setConfidenceThreshold()' instead")]] void setConfidenceThreshold(int confThr);

    /**
     * Rectify input images or not.
     */
    void setRectification(bool enable);

    /**
     * Computes and combines disparities in both L-R and R-L directions, and combine them.
     *
     * For better occlusion handling, discarding invalid disparity values
     */
    void setLeftRightCheck(bool enable);

    /**
     * Computes disparity with sub-pixel interpolation (5 fractional bits).
     *
     * Suitable for long range. Currently incompatible with extended disparity
     */
    void setSubpixel(bool enable);

    /**
     * Disparity range increased from 0-95 to 0-190, combined from full resolution and downscaled images.
     *
     * Suitable for short range objects. Currently incompatible with sub-pixel disparity
     */
    void setExtendedDisparity(bool enable);

    /**
     * Fill color for missing data at frame edges
     * @param color Grayscale 0..255, or -1 to replicate pixels
     */
    void setRectifyEdgeFillColor(int color);

    /**
     * Mirror rectified frames, only when LR-check mode is disabled. Default `true`.
     * The mirroring is required to have a normal non-mirrored disparity/depth output.
     *
     * A side effect of this option is disparity alignment to the perspective of left or right input:
     * `false`: mapped to left and mirrored, `true`: mapped to right.
     * With LR-check enabled, this option is ignored, none of the outputs are mirrored,
     * and disparity is mapped to right.
     *
     * @param enable True for normal disparity/depth, otherwise mirrored
     */
    void setRectifyMirrorFrame(bool enable);

    /**
     * Enable outputting rectified frames. Optimizes computation on device side when disabled.
     * DEPRECATED. The outputs are auto-enabled if used
     */
    [[deprecated("Function call should be removed")]] void setOutputRectified(bool enable);

    /**
     * Enable outputting 'depth' stream (converted from disparity).
     * In certain configurations, this will disable 'disparity' stream.
     * DEPRECATED. The output is auto-enabled if used
     */
    [[deprecated("Function call should be removed")]] void setOutputDepth(bool enable);

    /**
     * Useful for normalization of the disparity map.
     * @returns Maximum disparity value that the node can return
     */
    float getMaxDisparity() const;
};

}  // namespace node
}  // namespace dai
