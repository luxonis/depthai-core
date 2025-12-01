#pragma once

#include <depthai/common/DepthUnit.hpp>
#include <depthai/common/ProcessorType.hpp>
#include <depthai/common/optional.hpp>
#include <vector>

#include "depthai/pipeline/FilterParams.hpp"
#include "depthai/pipeline/datatype/Buffer.hpp"

namespace dai {

/**
 * StereoDepthConfig message.
 */
class StereoDepthConfig : public Buffer {
   public:
    /**
     * Construct StereoDepthConfig message.
     */
    StereoDepthConfig() = default;
    virtual ~StereoDepthConfig();

    using MedianFilter = filters::params::MedianFilter;

    struct AlgorithmControl {
        /**
         * Align the disparity/depth to the perspective of a rectified output, or center it
         */
        enum class DepthAlign : int32_t { RECTIFIED_RIGHT, RECTIFIED_LEFT, CENTER };

        using DepthUnit = dai::DepthUnit;

        /**
         * Set the disparity/depth alignment to the perspective of a rectified output, or center it
         */
        DepthAlign depthAlign = DepthAlign::RECTIFIED_LEFT;

        /**
         * Measurement unit for depth data.
         * Depth data is integer value, multiple of depth unit.
         */
        DepthUnit depthUnit = DepthUnit::MILLIMETER;

        /**
         * Custom depth unit multiplier, if custom depth unit is enabled, relative to 1 meter.
         * A multiplier of 1000 effectively means depth unit in millimeter.
         */
        float customDepthUnitMultiplier = 1000.f;

        /**
         * Computes and combines disparities in both L-R and R-L directions, and combine them.
         * For better occlusion handling
         */
        bool enableLeftRightCheck = true;

        /**
         * Enables software left right check. Applicable to RVC4 only.
         */
        bool enableSwLeftRightCheck = false;

        /**
         * Disparity range increased from 95 to 190, combined from full resolution and downscaled images.
         * Suitable for short range objects
         */
        bool enableExtended = false;

        /**
         * Computes disparity with sub-pixel interpolation (5 fractional bits), suitable for long range
         */
        bool enableSubpixel = true;

        /**
         * Left-right check threshold for left-right, right-left disparity map combine, 0..128
         * Used only when left-right check mode is enabled.
         * Defines the maximum difference between the confidence of pixels from left-right and right-left confidence maps
         */
        std::int32_t leftRightCheckThreshold = 10;

        /**
         * Number of fractional bits for subpixel mode
         *
         * Valid values: 3,4,5
         *
         * Defines the number of fractional disparities: 2^x
         *
         * Median filter postprocessing is supported only for 3 fractional bits
         */
        std::int32_t subpixelFractionalBits = 5;

        /**
         * Shift input frame by a number of pixels to increase minimum depth.
         * For example shifting by 48 will change effective disparity search range from (0,95] to [48,143].
         * An alternative approach to reducing the minZ.
         * We normally only recommend doing this when it is known that there will be no objects
         * farther away than MaxZ, such as having a depth camera mounted above a table
         * pointing down at the table surface.
         */
        std::int32_t disparityShift = 0;

        /*
         * Used only for debug purposes. centerAlignmentShiftFactor is set automatically in firmware,
         * from camera extrinsics when depth alignment to camera is enabled.
         * Center alignment is achieved by shifting the obtained disparity map by a scale factor.
         * It's used to align to a different camera that is on the same horizontal baseline as the two stereo cameras.
         * E.g. if we have a device with 10 cm stereo baseline, and we have another camera inbetween,
         * 9cm from the LEFT camera and 1 cm from the RIGHT camera we can align the obtained disparity map using a scale factor of 0.9.
         * Note that aligning disparity map to a different camera involves 2 steps:
         * 1. Shifting obtained disparity map.
         * 2. Warping the image to counter rotate and scaling to match the FOV.
         * Center alignment factor 1 is equivalent to RECTIFIED_RIGHT
         * Center alignment factor 0 is equivalent to RECTIFIED_LEFT
         */
        std::optional<float> centerAlignmentShiftFactor;

        /**
         * Invalidate X amount of pixels at the edge of disparity frame.
         * For right and center alignment X pixels will be invalidated from the right edge,
         * for left alignment from the left edge.
         */
        std::int32_t numInvalidateEdgePixels = 0;

        DEPTHAI_SERIALIZE(AlgorithmControl,
                          depthAlign,
                          depthUnit,
                          customDepthUnitMultiplier,
                          enableLeftRightCheck,
                          enableSwLeftRightCheck,
                          enableExtended,
                          enableSubpixel,
                          leftRightCheckThreshold,
                          subpixelFractionalBits,
                          disparityShift,
                          centerAlignmentShiftFactor,
                          numInvalidateEdgePixels);
    };

    struct ConfidenceMetrics {
        /**
         * Weight used with occlusion estimation to generate final confidence map.
         * Valid range is [0,32]
         */
        uint8_t occlusionConfidenceWeight = 20;
        /**
         * Weight used with local neighborhood motion vector variance estimation to generate final confidence map.
         * Valid range is [0,32].
         */
        uint8_t motionVectorConfidenceWeight = 4;
        /**
         * Threshold offset for MV variance in confidence generation. A value of 0 allows most variance.
         * Valid range is [0,3].
         */
        uint8_t motionVectorConfidenceThreshold = 1;
        /**
         * Weight used with flatness estimation to generate final confidence map.
         * Valid range is [0,32].
         */
        uint8_t flatnessConfidenceWeight = 8;
        /**
         * Threshold for flatness check in SGM block.
         * Valid range is [1,7].
         */
        uint8_t flatnessConfidenceThreshold = 2;
        /**
         * Flag to indicate whether final confidence value will be overidden by flatness value.
         * Valid range is {true,false}.
         */
        bool flatnessOverride = false;

        DEPTHAI_SERIALIZE(ConfidenceMetrics,
                          occlusionConfidenceWeight,
                          motionVectorConfidenceWeight,
                          motionVectorConfidenceThreshold,
                          flatnessConfidenceWeight,
                          flatnessConfidenceThreshold,
                          flatnessOverride);
    };

    /**
     * Post-processing filters, all the filters are applied in disparity domain.
     */
    struct PostProcessing {
        enum class Filter : int32_t { NONE = 0, DECIMATION, SPECKLE, MEDIAN, SPATIAL, TEMPORAL, FILTER_COUNT = TEMPORAL };

        /**
         * Order of filters to be applied if filtering is enabled.
         */
        std::array<Filter, 5> filteringOrder = {Filter::MEDIAN, Filter::DECIMATION, Filter::SPECKLE, Filter::SPATIAL, Filter::TEMPORAL};

        /**
         * Set kernel size for disparity/depth median filtering, or disable
         */
        MedianFilter median = MedianFilter::MEDIAN_OFF;

        /**
         * Sigma value for bilateral filter. 0 means disabled.
         * A larger value of the parameter means that farther colors within the pixel neighborhood will be mixed together.
         */
        std::int16_t bilateralSigmaValue = 0;

        using SpatialFilter = filters::params::SpatialFilter;

        /**
         * Edge-preserving filtering: This type of filter will smooth the depth noise while attempting to preserve edges.
         */
        SpatialFilter spatialFilter;

        using TemporalFilter = filters::params::TemporalFilter;

        /**
         * Temporal filtering with optional persistence.
         */
        TemporalFilter temporalFilter;

        /**
         * Threshold filtering.
         * Filters out distances outside of a given interval.
         */
        struct ThresholdFilter {
            /**
             * Minimum range in depth units.
             * Depth values under this value are invalidated.
             */
            std::int32_t minRange = 0;
            /**
             * Maximum range in depth units.
             * Depth values over this value are invalidated.
             */
            std::int32_t maxRange = 65535;

            DEPTHAI_SERIALIZE(ThresholdFilter, minRange, maxRange);
        };

        /**
         * Threshold filtering.
         * Filters out distances outside of a given interval.
         */
        ThresholdFilter thresholdFilter;

        /**
         * Brightness filtering.
         * If input frame pixel is too dark or too bright, disparity will be invalidated.
         * The idea is that for too dark/too bright pixels we have low confidence,
         * since that area was under/over exposed and details were lost.
         */
        struct BrightnessFilter {
            /**
             * Minimum pixel brightness.
             * If input pixel is less or equal than this value the depth value is invalidated.
             */
            std::int32_t minBrightness = 0;
            /**
             * Maximum range in depth units.
             * If input pixel is less or equal than this value the depth value is invalidated.
             */
            std::int32_t maxBrightness = 256;

            DEPTHAI_SERIALIZE(BrightnessFilter, minBrightness, maxBrightness);
        };

        /**
         * Brightness filtering.
         * If input frame pixel is too dark or too bright, disparity will be invalidated.
         * The idea is that for too dark/too bright pixels we have low confidence,
         * since that area was under/over exposed and details were lost.
         */
        BrightnessFilter brightnessFilter;

        using SpeckleFilter = filters::params::SpeckleFilter;

        /**
         * Speckle filtering.
         * Removes speckle noise.
         */
        SpeckleFilter speckleFilter;

        /**
         * Decimation filter.
         * Reduces the depth scene complexity. The filter runs on kernel sizes [2x2] to [8x8] pixels.
         */
        struct DecimationFilter {
            /**
             * Decimation factor.
             * Valid values are 1,2,3,4.
             * Disparity/depth map x/y resolution will be decimated with this value.
             */
            std::uint32_t decimationFactor = 1;
            /**
             * Decimation algorithm type.
             */
            enum class DecimationMode : int32_t {
                PIXEL_SKIPPING = 0,
                NON_ZERO_MEDIAN = 1,
                NON_ZERO_MEAN = 2,
            };
            /**
             * Decimation algorithm type.
             */
            DecimationMode decimationMode = DecimationMode::PIXEL_SKIPPING;

            DEPTHAI_SERIALIZE(DecimationFilter, decimationFactor, decimationMode);
        };

        /**
         * Decimation filter.
         * Reduces disparity/depth map x/y complexity, reducing runtime complexity for other filters.
         */
        DecimationFilter decimationFilter;

        struct HoleFilling {
            /**
             * Flag to enable post-processing hole-filling.
             */
            bool enable = true;

            /**
             * Pixels with confidence higher than this value are used to calculate an average disparity per superpixel.
             * Valid range is [1,255]
             */
            uint8_t highConfidenceThreshold = 210;
            /**
             * Pixels with confidence below this value will be filled with the average disparity of their corresponding superpixel.
             * Valid range is [1,255].
             */
            uint8_t fillConfidenceThreshold = 200;
            /**
             *    Represents the required percentange of pixels with confidence value
             *    above nHighConfThresh that are used to calculate average disparity per
             *    superpixel, where 1 means 50% or half, 2 means 25% or a quarter and 3
             *    means 12.5% or an eighth. If the required number of pixels are not
             *    found, the holes will not be filled.
             */
            uint8_t minValidDisparity = 1;  // todo enum: 50%, 25%, 12.5%
            /**
             * If enabled, sets to 0 the disparity of pixels with confidence below
             * nFillConfThresh, which did not pass nMinValidPixels criteria.
             * Valid range is {true, false}.
             */
            bool invalidateDisparities = true;

            DEPTHAI_SERIALIZE(HoleFilling, enable, highConfidenceThreshold, fillConfidenceThreshold, minValidDisparity, invalidateDisparities);
        };

        HoleFilling holeFilling;

        struct AdaptiveMedianFilter {
            /**
             * Flag to enable adaptive median filtering for a final pass of filtering on low confidence pixels.
             */
            bool enable = true;

            /**
             * Confidence threshold for adaptive median filtering.
             * Should be less than nFillConfThresh value used in evaDfsHoleFillConfig.
             * Valid range is [0,255].
             */
            uint8_t confidenceThreshold = 200;

            DEPTHAI_SERIALIZE(AdaptiveMedianFilter, enable, confidenceThreshold);
        };

        AdaptiveMedianFilter adaptiveMedianFilter;

        DEPTHAI_SERIALIZE(PostProcessing,
                          filteringOrder,
                          median,
                          bilateralSigmaValue,
                          spatialFilter,
                          temporalFilter,
                          thresholdFilter,
                          brightnessFilter,
                          speckleFilter,
                          decimationFilter,
                          holeFilling,
                          adaptiveMedianFilter);
    };

    /**
     * The basic cost function used by the Stereo Accelerator for matching the left and right images is the Census
     * Transform. It works on a block of pixels and computes a bit vector which represents the structure of the
     * image in that block.
     * There are two types of Census Transform based on how the middle pixel is used:
     * Classic Approach and Modified Census. The comparisons that are made between pixels can be or not thresholded.
     * In some cases a mask can be applied to filter out only specific bits from the entire bit stream.
     * All these approaches are:
     * Classic Approach: Uses middle pixel to compare against all its neighbors over a defined window. Each
     * comparison results in a new bit, that is 0 if central pixel is smaller, or 1 if is it bigger than its neighbor.
     * Modified Census Transform: same as classic Census Transform, but instead of comparing central pixel
     * with its neighbors, the window mean will be compared with each pixel over the window.
     * Thresholding Census Transform: same as classic Census Transform, but it is not enough that a
     * neighbor pixel to be bigger than the central pixel, it must be significant bigger (based on a threshold).
     * Census Transform with Mask: same as classic Census Transform, but in this case not all of the pixel from
     * the support window are part of the binary descriptor. We use a ma sk “M” to define which pixels are part
     * of the binary descriptor (1), and which pixels should be skipped (0).
     */
    struct CensusTransform {
        /**
         * Census transform kernel size possible values.
         */
        enum class KernelSize : std::int32_t { AUTO = -1, KERNEL_5x5 = 0, KERNEL_7x7, KERNEL_7x9 };

        /**
         * Census transform kernel size.
         */
        KernelSize kernelSize = KernelSize::AUTO;

        /**
         * Census transform mask, default - auto, mask is set based on resolution and kernel size.
         * Disabled for 400p input resolution.
         * Enabled for 720p.
         * 0XA82415 for 5x5 census transform kernel.
         * 0XAA02A8154055 for 7x7 census transform kernel.
         * 0X2AA00AA805540155 for 7x9 census transform kernel.
         * Empirical values.
         */
        uint64_t kernelMask = 0;

        /**
         * If enabled, each pixel in the window is compared with the mean window value instead of the central pixel.
         */
        bool enableMeanMode = true;

        /**
         * Census transform comparison threshold value.
         */
        uint32_t threshold = 0;

        /**
         * Used to reduce small fixed levels of noise across all luminance values
         * in the current image.
         * Valid range is [0,127]. Default value is 0.
         */
        int8_t noiseThresholdOffset = 1;
        /**
         * Used to reduce noise values that increase with luminance in the
         * current image.
         * Valid range is [-128,127]. Default value is 0.
         */
        int8_t noiseThresholdScale = 1;

        DEPTHAI_SERIALIZE(CensusTransform, kernelSize, kernelMask, enableMeanMode, threshold, noiseThresholdOffset, noiseThresholdScale);
    };

    /**
     * The matching cost is way of measuring the similarity of image locations in stereo correspondence
     * algorithm. Based on the configuration parameters and based on the descriptor type, a linear equation
     * is applied to computing the cost for each candidate disparity at each pixel.
     */
    struct CostMatching {
        /**
         * Disparity search range: 64 or 96 pixels are supported by the HW.
         */
        enum class DisparityWidth : std::uint32_t { DISPARITY_64, DISPARITY_96 };

        /**
         * Disparity search range, default 96 pixels.
         */
        DisparityWidth disparityWidth = DisparityWidth::DISPARITY_96;

        /**
         * Disparity companding using sparse matching.
         * Matching pixel by pixel for N disparities.
         * Matching every 2nd pixel for M disparitites.
         * Matching every 4th pixel for T disparities.
         * In case of 96 disparities: N=48, M=32, T=16.
         * This way the search range is extended to 176 disparities, by sparse matching.
         * Note: when enabling this flag only depth map will be affected, disparity map is not.
         */
        bool enableCompanding = false;

        /**
         * Used only for debug purposes, SW postprocessing handled only invalid value of 0 properly.
         */
        uint8_t invalidDisparityValue = 0;

        /**
         * Disparities with confidence value over this threshold are accepted.
         */
        uint8_t confidenceThreshold = 55;

        /**
         * Enable software confidence thresholding. Applicable to RVC4 only.
         */
        bool enableSwConfidenceThresholding = false;

        /**
         * The linear equation applied for computing the cost is:
         * COMB_COST = α*AD + β*(CTC<<3).
         * CLAMP(COMB_COST >> 5, threshold).
         * Where AD is the Absolute Difference between 2 pixels values.
         * CTC is the Census Transform Cost between 2 pixels, based on Hamming distance (xor).
         * The α and β parameters are subject to fine tuning by the user.
         */
        struct LinearEquationParameters {
            uint8_t alpha = 0;
            uint8_t beta = 2;
            uint8_t threshold = 127;

            DEPTHAI_SERIALIZE(LinearEquationParameters, alpha, beta, threshold);
        };

        /**
         * Cost calculation linear equation parameters.
         */
        LinearEquationParameters linearEquationParameters;

        DEPTHAI_SERIALIZE(CostMatching,
                          disparityWidth,
                          enableCompanding,
                          invalidDisparityValue,
                          confidenceThreshold,
                          enableSwConfidenceThresholding,
                          linearEquationParameters);
    };

    /**
     * Cost Aggregation is based on Semi Global Block Matching (SGBM). This algorithm uses a semi global
     * technique to aggregate the cost map. Ultimately the idea is to build inertia into the stereo algorithm. If
     * a pixel has very little texture information, then odds are the correct disparity for this pixel is close to
     * that of the previous pixel considered. This means that we get improved results in areas with low
     * texture.
     */
    struct CostAggregation {
        static constexpr const int defaultPenaltyP1 = 250;
        static constexpr const int defaultPenaltyP2 = 500;

        /**
         * Cost calculation linear equation parameters.
         */
        uint8_t divisionFactor = 1;

        /**
         * Horizontal P1 penalty cost parameter.
         */
        uint16_t horizontalPenaltyCostP1 = defaultPenaltyP1;
        /**
         * Horizontal P2 penalty cost parameter.
         */
        uint16_t horizontalPenaltyCostP2 = defaultPenaltyP2;

        /**
         * Vertical P1 penalty cost parameter.
         */
        uint16_t verticalPenaltyCostP1 = defaultPenaltyP1;
        /**
         * Vertical P2 penalty cost parameter.
         */
        uint16_t verticalPenaltyCostP2 = defaultPenaltyP2;

        /**
         * Structure for adaptive P1 penalty configuration.
         */
        struct P1Config {
            /**
             * Used to disable/enable adaptive penalty.
             */
            bool enableAdaptive = true;
            /**
             * Used as the default penalty value when nAdapEnable is disabled.
             * A bigger value enforces higher smoothness and reduced noise at the cost of lower edge accuracy.
             * This value must be smaller than P2 default penalty.
             * Valid range is [10,50].
             */
            uint8_t defaultValue = 11;
            /**
             * Penalty value on edges when nAdapEnable is enabled.
             * A smaller penalty value permits higher change in disparity.
             * This value must be smaller than or equal to P2 edge penalty.
             * Valid range is [10,50].
             */
            uint8_t edgeValue = 10;
            /**
             * Penalty value on low texture regions when nAdapEnable is enabled.
             * A smaller penalty value permits higher change in disparity.
             * This value must be smaller than or equal to P2 smoothness penalty.
             * Valid range is [10,50].
             */
            uint8_t smoothValue = 22;
            /**
             * Threshold value on edges when nAdapEnable is enabled.
             * A bigger value permits higher neighboring feature dissimilarity tolerance.
             * This value is shared with P2 penalty configuration.
             * Valid range is [8,16].
             */
            uint8_t edgeThreshold = 15;
            /**
             * Threshold value on low texture regions when nAdapEnable is enabled.
             * A bigger value permits higher neighboring feature dissimilarity tolerance.
             * This value is shared with P2 penalty configuration.
             * Valid range is [2,12].
             */
            uint8_t smoothThreshold = 5;

            DEPTHAI_SERIALIZE(P1Config, enableAdaptive, defaultValue, edgeValue, smoothValue, edgeThreshold, smoothThreshold);
        };

        P1Config p1Config;

        /**
         * Structure for adaptive P2 penalty configuration.
         */
        struct P2Config {
            /**
             * Used to disable/enable adaptive penalty.
             */
            bool enableAdaptive = true;
            /**
             * Used as the default penalty value when nAdapEnable is disabled.
             * A bigger value enforces higher smoothness and reduced noise at the cost of lower edge accuracy.
             * This value must be larger than P1 default penalty.
             * Valid range is [20,100].
             */
            uint8_t defaultValue = 33;
            /**
             * Penalty value on edges when nAdapEnable is enabled.
             * A smaller penalty value permits higher change in disparity.
             * This value must be larger than or equal to P1 edge penalty.
             * Valid range is [20,100].
             */
            uint8_t edgeValue = 22;
            /**
             * Penalty value on low texture regions when nAdapEnable is enabled.
             * A smaller penalty value permits higher change in disparity.
             * This value must be larger than or equal to P1 smoothness penalty.
             * Valid range is [20,100].
             */
            uint8_t smoothValue = 63;

            DEPTHAI_SERIALIZE(P2Config, enableAdaptive, defaultValue, edgeValue, smoothValue);
        };

        P2Config p2Config;

        DEPTHAI_SERIALIZE(CostAggregation,
                          divisionFactor,
                          horizontalPenaltyCostP1,
                          horizontalPenaltyCostP2,
                          verticalPenaltyCostP1,
                          verticalPenaltyCostP2,
                          p1Config,
                          p2Config);
    };

    /**
     * @param align Set the disparity/depth alignment: centered (between the 'left' and 'right' inputs),
     * or from the perspective of a rectified output stream
     */
    StereoDepthConfig& setDepthAlign(AlgorithmControl::DepthAlign align);

    /**
     * Confidence threshold for disparity calculation
     * @param confThr Confidence threshold value 0..255
     */
    StereoDepthConfig& setConfidenceThreshold(int confThr);
    /**
     * Get confidence threshold for disparity calculation
     */
    int getConfidenceThreshold() const;

    /**
     * @param median Set kernel size for disparity/depth median filtering, or disable
     */
    StereoDepthConfig& setMedianFilter(MedianFilter median);
    /**
     * Get median filter setting
     */
    MedianFilter getMedianFilter() const;

    /**
     * A larger value of the parameter means that farther colors within the pixel neighborhood will be mixed together,
     * resulting in larger areas of semi-equal color.
     * @param sigma Set sigma value for 5x5 bilateral filter. 0..65535
     */
    StereoDepthConfig& setBilateralFilterSigma(uint16_t sigma);
    /**
     * Get sigma value for 5x5 bilateral filter
     */
    uint16_t getBilateralFilterSigma() const;

    /**
     * @param threshold Set threshold for left-right, right-left disparity map combine, 0..255
     */
    StereoDepthConfig& setLeftRightCheckThreshold(int threshold);
    /**
     * Get threshold for left-right check combine
     */
    int getLeftRightCheckThreshold() const;

    /**
     * Computes and combines disparities in both L-R and R-L directions, and combine them.
     *
     * For better occlusion handling, discarding invalid disparity values
     */
    StereoDepthConfig& setLeftRightCheck(bool enable);

    /**
     * Get left-right check setting
     */
    bool getLeftRightCheck() const;

    /**
     * Disparity range increased from 95 to 190, combined from full resolution and downscaled images.
     * Suitable for short range objects
     */
    StereoDepthConfig& setExtendedDisparity(bool enable);

    /**
     * Get extended disparity setting
     */
    bool getExtendedDisparity() const;

    /**
     * Computes disparity with sub-pixel interpolation (3 fractional bits by default).
     *
     * Suitable for long range. Currently incompatible with extended disparity
     */
    StereoDepthConfig& setSubpixel(bool enable);

    /**
     * Get subpixel setting
     */
    bool getSubpixel() const;

    /**
     * Number of fractional bits for subpixel mode.
     * Default value: 3.
     * Valid values: 3,4,5.
     * Defines the number of fractional disparities: 2^x.
     * Median filter postprocessing is supported only for 3 fractional bits.
     */
    StereoDepthConfig& setSubpixelFractionalBits(int subpixelFractionalBits);

    /**
     * Get number of fractional bits for subpixel mode
     */
    int getSubpixelFractionalBits() const;

    /**
     * Set depth unit of depth map.
     *
     * Meter, centimeter, millimeter, inch, foot or custom unit is available.
     */
    StereoDepthConfig& setDepthUnit(AlgorithmControl::DepthUnit depthUnit);

    /**
     * Get depth unit of depth map.
     */
    AlgorithmControl::DepthUnit getDepthUnit();

    /**
     * Set custom depth unit multiplier relative to 1 meter.
     */
    StereoDepthConfig& setCustomDepthUnitMultiplier(float multiplier);

    /**
     * Get custom depth unit multiplier relative to 1 meter.
     */
    float getCustomDepthUnitMultiplier() const;

    /**
     * Shift input frame by a number of pixels to increase minimum depth.
     * For example shifting by 48 will change effective disparity search range from (0,95] to [48,143].
     * An alternative approach to reducing the minZ.
     * We normally only recommend doing this when it is known that there will be no objects
     * farther away than MaxZ, such as having a depth camera mounted above a table
     * pointing down at the table surface.
     */
    StereoDepthConfig& setDisparityShift(int disparityShift);

    /**
     * Invalidate X amount of pixels at the edge of disparity frame.
     * For right and center alignment X pixels will be invalidated from the right edge,
     * for left alignment from the left edge.
     */
    StereoDepthConfig& setNumInvalidateEdgePixels(int32_t numInvalidateEdgePixels);

    /**
     * Set filters compute backend
     */
    StereoDepthConfig& setFiltersComputeBackend(dai::ProcessorType filtersBackend);

    /**
     * Get filters compute backend
     */
    dai::ProcessorType getFiltersComputeBackend() const;

    /**
     * Useful for normalization of the disparity map.
     * @returns Maximum disparity value that the node can return
     */
    float getMaxDisparity() const;

    /**
     * Controls the flow of stereo algorithm - left-right check, subpixel etc.
     */
    AlgorithmControl algorithmControl;

    /**
     * Controls the postprocessing of disparity and/or depth map.
     */
    PostProcessing postProcessing;

    /**
     * Census transform settings.
     */
    CensusTransform censusTransform;

    /**
     * Cost matching settings.
     */
    CostMatching costMatching;

    /**
     * Cost aggregation settings.
     */
    CostAggregation costAggregation;

    /**
     * Confidence metrics settings.
     */
    ConfidenceMetrics confidenceMetrics;

    dai::ProcessorType filtersBackend = dai::ProcessorType::CPU;

    void serialize(std::vector<std::uint8_t>& metadata, DatatypeEnum& datatype) const override;
    DatatypeEnum getDatatype() const override {
        return DatatypeEnum::StereoDepthConfig;
    }
    DEPTHAI_SERIALIZE(StereoDepthConfig, algorithmControl, postProcessing, censusTransform, costMatching, costAggregation, confidenceMetrics, filtersBackend);
};

}  // namespace dai
