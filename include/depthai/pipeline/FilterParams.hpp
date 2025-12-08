#pragma once

#include <cstdint>

#include "depthai/utility/Serialization.hpp"

namespace dai {
namespace filters {
namespace params {

enum class MedianFilter : int32_t { MEDIAN_OFF = 0, KERNEL_3x3 = 3, KERNEL_5x5 = 5, KERNEL_7x7 = 7 };

/**
 * 1D edge-preserving spatial filter using high-order domain transform.
 */
struct SpatialFilter {
    static constexpr const std::int32_t DEFAULT_DELTA_VALUE = 3;

    /**
     * Whether to enable or disable the filter.
     */
    bool enable = false;

    /**
     * An in-place heuristic symmetric hole-filling mode applied horizontally during the filter passes.
     * Intended to rectify minor artefacts with minimal performance impact.
     * Search radius for hole filling.
     */
    std::uint8_t holeFillingRadius = 2;

    /**
     * The Alpha factor in an exponential moving average with Alpha=1 - no filter. Alpha = 0 - infinite filter.
     * Determines the amount of smoothing.
     */
    float alpha = 0.5f;

    /**
     * Step-size boundary. Establishes the threshold used to preserve "edges".
     * If the disparity value between neighboring pixels exceed the disparity threshold set by this delta parameter,
     * then filtering will be temporarily disabled.
     * Default value 0 means auto: 3 disparity integer levels.
     * In case of subpixel mode it's 3*number of subpixel levels.
     */
    std::int32_t delta = DEFAULT_DELTA_VALUE;

    /**
     * Number of iterations over the image in both horizontal and vertical direction.
     */
    std::int32_t numIterations = 1;

    DEPTHAI_SERIALIZE(SpatialFilter, enable, holeFillingRadius, alpha, delta, numIterations);
};

/**
 * Temporal filtering with optional persistence.
 */
struct TemporalFilter {
    static constexpr const std::int32_t DEFAULT_DELTA_VALUE = 3;

    /**
     * Whether to enable or disable the filter.
     */
    bool enable = false;

    /**
     * Persistency algorithm type.
     */
    enum class PersistencyMode : int32_t {
        PERSISTENCY_OFF = 0,
        VALID_8_OUT_OF_8 = 1,
        VALID_2_IN_LAST_3 = 2,
        VALID_2_IN_LAST_4 = 3,
        VALID_2_OUT_OF_8 = 4,
        VALID_1_IN_LAST_2 = 5,
        VALID_1_IN_LAST_5 = 6,
        VALID_1_IN_LAST_8 = 7,
        PERSISTENCY_INDEFINITELY = 8,
    };

    /**
     * Persistency mode.
     * If the current disparity/depth value is invalid, it will be replaced by an older value, based on persistency mode.
     */
    PersistencyMode persistencyMode = PersistencyMode::VALID_2_IN_LAST_4;

    /**
     * The Alpha factor in an exponential moving average with Alpha=1 - no filter. Alpha = 0 - infinite filter.
     * Determines the extent of the temporal history that should be averaged.
     */
    float alpha = 0.4f;

    /**
     * Step-size boundary. Establishes the threshold used to preserve surfaces (edges).
     * If the disparity value between neighboring pixels exceed the disparity threshold set by this delta parameter,
     * then filtering will be temporarily disabled.
     * Default value 0 means auto: 3 disparity integer levels.
     * In case of subpixel mode it's 3*number of subpixel levels.
     */
    std::int32_t delta = DEFAULT_DELTA_VALUE;

    DEPTHAI_SERIALIZE(TemporalFilter, enable, persistencyMode, alpha, delta);
};

/**
 * Speckle filtering.
 * Removes speckle noise.
 */
struct SpeckleFilter {
    /**
     * Whether to enable or disable the filter.
     */
    bool enable = false;
    /**
     * Speckle search range.
     */
    std::uint32_t speckleRange = 50;

    /**
     * Maximum difference between neighbor disparity pixels to put them into the same blob.
     * Units in disparity integer levels.
     */
    std::uint32_t differenceThreshold = 2;

    DEPTHAI_SERIALIZE(SpeckleFilter, enable, speckleRange, differenceThreshold);
};

}  // namespace params
}  // namespace filters
}  // namespace dai