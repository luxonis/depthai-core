#pragma once

#include <depthai/pipeline/DeviceNode.hpp>
#include <memory>

// shared
#include "depthai/pipeline/datatype/StereoDepthConfig.hpp"
#include "depthai/properties/StereoDepthProperties.hpp"

namespace dai {
namespace node {

/**
 * @brief StereoDepth node. Compute stereo disparity and depth from left-right image pair.
 */
class StereoDepth : public DeviceNodeCRTP<DeviceNode, StereoDepth, StereoDepthProperties> {
   public:
    constexpr static const char* NAME = "StereoDepth";
    using DeviceNodeCRTP::DeviceNodeCRTP;

    /**
     * Preset modes for stereo depth.
     */
    enum class PresetMode : std::uint32_t {
        /**
         * Prefers accuracy over density. More invalid depth values, but less outliers.
         */
        HIGH_ACCURACY,
        /**
         * Prefers density over accuracy. Less invalid depth values, but more outliers.
         */
        HIGH_DENSITY
    };
    std::shared_ptr<StereoDepth> build(Node::Output& left, Node::Output& right, PresetMode presetMode = PresetMode::HIGH_ACCURACY) {
        this->presetMode = presetMode;
        left.link(this->left);
        right.link(this->right);
        return std::static_pointer_cast<StereoDepth>(shared_from_this());
    }

    std::shared_ptr<StereoDepth> build(bool autoCreateCameras, PresetMode presetMode = PresetMode::HIGH_ACCURACY);

   protected:
    Properties& getProperties();
    StereoDepth() = default;
    StereoDepth(std::unique_ptr<Properties> props);

   private:
    PresetMode presetMode = PresetMode::HIGH_ACCURACY;

   public:
    using MedianFilter = dai::StereoDepthConfig::MedianFilter;

    /**
     * Initial config to use for StereoDepth.
     */
    StereoDepthConfig initialConfig;

    /**
     * Input StereoDepthConfig message with ability to modify parameters in runtime.
     */
    Input inputConfig{
        *this, {"inputConfig", DEFAULT_GROUP, DEFAULT_BLOCKING, DEFAULT_QUEUE_SIZE, {{{DatatypeEnum::StereoDepthConfig, false}}}, DEFAULT_WAIT_FOR_MESSAGE}};

    /**
     * Input for left ImgFrame of left-right pair
     */
    Input left{*this, {"left", DEFAULT_GROUP, DEFAULT_BLOCKING, DEFAULT_QUEUE_SIZE, {{{DatatypeEnum::ImgFrame, true}}}, DEFAULT_WAIT_FOR_MESSAGE}};

    /**
     * Input for right ImgFrame of left-right pair
     */
    Input right{*this, {"right", DEFAULT_GROUP, DEFAULT_BLOCKING, DEFAULT_QUEUE_SIZE, {{{DatatypeEnum::ImgFrame, true}}}, DEFAULT_WAIT_FOR_MESSAGE}};

    // TODO(before mainline) - API not supported on RVC2
    /**
     * Input pixel descriptor for left ImgFrame.
     * Input type must be 4 bytes per pixel
     */
    Input inputLeftPixelDescriptor{
        *this, {"inputLeftPixelDescriptor", DEFAULT_GROUP, DEFAULT_BLOCKING, DEFAULT_QUEUE_SIZE, {{{DatatypeEnum::ImgFrame, true}}}, DEFAULT_WAIT_FOR_MESSAGE}};

    // TODO(before mainline) - API not supported on RVC2
    /**
     * Input pixel descriptor for right ImgFrame.
     * Input type must be 4 bytes per pixel
     */
    Input inputRightPixelDescriptor{
        *this,
        {"inputRightPixelDescriptor", DEFAULT_GROUP, DEFAULT_BLOCKING, DEFAULT_QUEUE_SIZE, {{{DatatypeEnum::ImgFrame, true}}}, DEFAULT_WAIT_FOR_MESSAGE}};

    /**
     * Outputs ImgFrame message that carries RAW16 encoded (0..65535) depth data in depth units (millimeter by default).
     *
     * Non-determined / invalid depth values are set to 0
     */
    Output depth{*this, {"depth", DEFAULT_GROUP, {{{DatatypeEnum::ImgFrame, false}}}}};

    /**
     * Outputs ImgFrame message that carries RAW8 / RAW16 encoded disparity data:
     * RAW8 encoded (0..95) for standard mode;
     * RAW8 encoded (0..190) for extended disparity mode;
     * RAW16 encoded for subpixel disparity mode:
     * - 0..760 for 3 fractional bits (by default)
     * - 0..1520 for 4 fractional bits
     * - 0..3040 for 5 fractional bits
     */
    Output disparity{*this, {"disparity", DEFAULT_GROUP, {{{DatatypeEnum::ImgFrame, false}}}}};

    /**
     * Passthrough ImgFrame message from 'left' Input.
     */
    Output syncedLeft{*this, {"syncedLeft", DEFAULT_GROUP, {{{DatatypeEnum::ImgFrame, false}}}}};

    /**
     * Passthrough ImgFrame message from 'right' Input.
     */
    Output syncedRight{*this, {"syncedRight", DEFAULT_GROUP, {{{DatatypeEnum::ImgFrame, false}}}}};

    /**
     * Outputs ImgFrame message that carries RAW8 encoded (grayscale) rectified frame data.
     */
    Output rectifiedLeft{*this, {"rectifiedLeft", DEFAULT_GROUP, {{{DatatypeEnum::ImgFrame, false}}}}};

    /**
     * Outputs ImgFrame message that carries RAW8 encoded (grayscale) rectified frame data.
     */
    Output rectifiedRight{*this, {"rectifiedRight", DEFAULT_GROUP, {{{DatatypeEnum::ImgFrame, false}}}}};

    /**
     * Outputs StereoDepthConfig message that contains current stereo configuration.
     */
    Output outConfig{*this, {"outConfig", DEFAULT_GROUP, {{{DatatypeEnum::StereoDepthConfig, false}}}}};

    /**
     * Outputs ImgFrame message that carries left-right check first iteration (before combining with second iteration) disparity map.
     * Useful for debugging/fine tuning.
     */
    Output debugDispLrCheckIt1{*this, {"debugDispLrCheckIt1", DEFAULT_GROUP, {{{DatatypeEnum::ImgFrame, false}}}}};

    /**
     * Outputs ImgFrame message that carries left-right check second iteration (before combining with first iteration) disparity map.
     * Useful for debugging/fine tuning.
     */
    Output debugDispLrCheckIt2{*this, {"debugDispLrCheckIt2", DEFAULT_GROUP, {{{DatatypeEnum::ImgFrame, false}}}}};

    /**
     * Outputs ImgFrame message that carries extended left-right check first iteration (downscaled frame, before combining with second iteration) disparity map.
     * Useful for debugging/fine tuning.
     */
    Output debugExtDispLrCheckIt1{*this, {"debugExtDispLrCheckIt1", DEFAULT_GROUP, {{{DatatypeEnum::ImgFrame, false}}}}};

    /**
     * Outputs ImgFrame message that carries extended left-right check second iteration (downscaled frame, before combining with first iteration) disparity map.
     * Useful for debugging/fine tuning.
     */
    Output debugExtDispLrCheckIt2{*this, {"debugExtDispLrCheckIt2", DEFAULT_GROUP, {{{DatatypeEnum::ImgFrame, false}}}}};

    /**
     * Outputs ImgFrame message that carries cost dump of disparity map.
     * Useful for debugging/fine tuning.
     */
    Output debugDispCostDump{*this, {"debugDispCostDump", DEFAULT_GROUP, {{{DatatypeEnum::ImgFrame, false}}}}};

    /**
     * Outputs ImgFrame message that carries RAW8 confidence map.
     * Lower values mean lower confidence of the calculated disparity value.
     * RGB alignment, left-right check or any postprocessing (e.g., median filter) is not performed on confidence map.
     */
    Output confidenceMap{*this, {"confidenceMap", DEFAULT_GROUP, {{{DatatypeEnum::ImgFrame, false}}}}};

    // TODO(before mainline) - API not supported on RVC2
    Output pixelDescriptorsLeft{*this, {"pixelDescriptorsLeft", DEFAULT_GROUP, {{{DatatypeEnum::ImgFrame, false}}}}};

    // TODO(before mainline) - API not supported on RVC2
    Output pixelDescriptorsRight{*this, {"pixelDescriptorsRight", DEFAULT_GROUP, {{{DatatypeEnum::ImgFrame, false}}}}};

    /**
     * Specify that a passthrough/dummy calibration should be used,
     * when input frames are already rectified (e.g. sourced from recordings on the host)
     */
    [[deprecated("Use 'Stereo::setRectification(false)' instead")]] void setEmptyCalibration();

    /**
     * Specify local filesystem paths to the mesh calibration files for 'left' and 'right' inputs.
     *
     * When a mesh calibration is set, it overrides the camera intrinsics/extrinsics matrices.
     * Overrides useHomographyRectification behavior.
     * Mesh format: a sequence of (y,x) points as 'float' with coordinates from the input image
     * to be mapped in the output. The mesh can be subsampled, configured by `setMeshStep`.
     *
     * With a 1280x800 resolution and the default (16,16) step, the required mesh size is:
     *
     * width: 1280 / 16 + 1 = 81
     *
     * height: 800 / 16 + 1 = 51
     */
    void loadMeshFiles(const dai::Path& pathLeft, const dai::Path& pathRight);

    /**
     * Specify mesh calibration data for 'left' and 'right' inputs, as vectors of bytes.
     * Overrides useHomographyRectification behavior.
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
     * Specify input resolution size
     *
     * Optional if MonoCamera exists, otherwise necessary
     */
    void setInputResolution(std::tuple<int, int> resolution);

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
    [[deprecated("Use 'initialConfig.setMedianFilter()' instead")]] void setMedianFilter(MedianFilter median);

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
     * Computes disparity with sub-pixel interpolation (3 fractional bits by default).
     *
     * Suitable for long range. Currently incompatible with extended disparity
     */
    void setSubpixel(bool enable);

    /**
     * Number of fractional bits for subpixel mode.
     * Default value: 3.
     * Valid values: 3,4,5.
     * Defines the number of fractional disparities: 2^x.
     * Median filter postprocessing is supported only for 3 fractional bits.
     */
    void setSubpixelFractionalBits(int subpixelFractionalBits);

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
     * DEPRECATED function. It was removed, since rectified images are not flipped anymore.
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
    [[deprecated("Function call should be removed")]] void setRectifyMirrorFrame(bool enable);

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
     * Enable runtime stereo mode switch, e.g. from standard to LR-check.
     * Note: when enabled resources allocated for worst case to enable switching to any mode.
     */
    void setRuntimeModeSwitch(bool enable);

    /**
     * Specify number of frames in pool.
     * @param numFramesPool How many frames should the pool have
     */
    void setNumFramesPool(int numFramesPool);

    /**
     * Useful for normalization of the disparity map.
     * @returns Maximum disparity value that the node can return
     */
    [[deprecated("Use 'initialConfig.getMaxDisparity()' instead")]] float getMaxDisparity() const;

    /**
     * Specify allocated hardware resources for stereo depth.
     * Suitable only to increase post processing runtime.
     * @param numShaves Number of shaves.
     * @param numMemorySlices Number of memory slices.
     */
    void setPostProcessingHardwareResources(int numShaves, int numMemorySlices);

    /**
     * Sets a default preset based on specified option.
     * @param mode Stereo depth preset mode
     */
    void setDefaultProfilePreset(PresetMode mode);

    /**
     * Whether to use focal length from calibration intrinsics or calculate based on calibration FOV.
     * Default value is true.
     */
    [[deprecated("setFocalLengthFromCalibration is deprecated. Default value is true.")]] void setFocalLengthFromCalibration(bool focalLengthFromCalibration);

    /**
     * Use 3x3 homography matrix for stereo rectification instead of sparse mesh generated on device.
     * Default behaviour is AUTO, for lenses with FOV over 85 degrees sparse mesh is used, otherwise 3x3 homography.
     * If custom mesh data is provided through loadMeshData or loadMeshFiles this option is ignored.
     * @param useHomographyRectification true: 3x3 homography matrix generated from calibration data is used for stereo rectification, can't correct lens
     * distortion.
     * false: sparse mesh is generated on-device from calibration data with mesh step specified with setMeshStep (Default: (16, 16)), can correct lens
     * distortion. Implementation for generating the mesh is same as opencv's initUndistortRectifyMap function. Only the first 8 distortion coefficients are
     * used from calibration data.
     */
    void useHomographyRectification(bool useHomographyRectification);

    // TODO(before mainline) - API not supported on RVC2
    /**
     * Whether to perform vertical stereo matching or not.
     * Default value is false.
     * If set to true rectification process includes 90 degree clock wise rotation to perform vertical matching.
     */
    void setVerticalStereo(bool verticalStereo);

    // TODO(before mainline) - API not supported on RVC2
    /**
     * Whether to use custom pixel descriptors sent from host to device for debugging purposes.
     * Default value is false.
     */
    void setCustomPixelDescriptors(bool customPixelDescriptors);

    /**
     * Equivalent to useHomographyRectification(!enableDistortionCorrection)
     */
    void enableDistortionCorrection(bool enableDistortionCorrection);

    /**
     * Whether to enable frame syncing inside stereo node or not. Suitable if inputs are known to be synced.
     */
    void setFrameSync(bool enableFrameSync);
    // TODO(before mainline) - API not supported on RVC3
    /**
     * Override baseline from calibration.
     * Used only in disparity to depth conversion.
     * Units are centimeters.
     */
    void setBaseline(float baseline);

    // TODO(before mainline) - API not supported on RVC3
    /**
     * Override focal length from calibration.
     * Used only in disparity to depth conversion.
     * Units are pixels.
     */
    void setFocalLength(float focalLength);

    // TODO(before mainline) - API not supported on RVC3
    /**
     * Use baseline information for disparity to depth conversion from specs (design data) or from calibration.
     * Default: true
     */
    void setDisparityToDepthUseSpecTranslation(bool specTranslation);

    // TODO(before mainline) - API not supported on RVC3
    /**
     * Obtain rectification matrices using spec translation (design data) or from calibration in calculations.
     * Should be used only for debugging.
     * Default: false
     */
    void setRectificationUseSpecTranslation(bool specTranslation);

    // TODO(before mainline) - API not supported on RVC3
    /**
     * Use baseline information for depth alignment from specs (design data) or from calibration.
     * Default: true
     */
    void setDepthAlignmentUseSpecTranslation(bool specTranslation);

    // TODO(before mainline) - API not supported on RVC3
    /**
     * Free scaling parameter between 0 (when all the pixels in the undistorted image are valid)
     * and 1 (when all the source image pixels are retained in the undistorted image).
     * On some high distortion lenses, and/or due to rectification (image rotated) invalid areas may appear even with alpha=0,
     * in these cases alpha < 0.0 helps removing invalid areas.
     * See getOptimalNewCameraMatrix from opencv for more details.
     */
    void setAlphaScaling(float alpha);
};

}  // namespace node
}  // namespace dai
