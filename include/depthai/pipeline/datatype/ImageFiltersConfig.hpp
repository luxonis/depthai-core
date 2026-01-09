#pragma once

#include <depthai/pipeline/datatype/StereoDepthConfig.hpp>

#include "depthai/common/variant.hpp"
#include "depthai/pipeline/datatype/Buffer.hpp"

namespace dai {

using MedianFilterParams = dai::filters::params::MedianFilter;
using SpatialFilterParams = dai::filters::params::SpatialFilter;
using SpeckleFilterParams = dai::filters::params::SpeckleFilter;
using TemporalFilterParams = dai::filters::params::TemporalFilter;

/**
 * @brief Preset modes for ImageFiltersConfig.
 */
enum class ImageFiltersPresetMode : std::uint32_t {
    /// Optimized for low range ToF measurements (0.2m–2m).
    TOF_LOW_RANGE,

    /// Optimized for mid range ToF measurements (e.g., 0.2m–5m).
    TOF_MID_RANGE,

    /// Optimized for high range ToF measurements (e.g., 1m–6m).
    TOF_HIGH_RANGE,
};

// union of all filter params
typedef std::variant<MedianFilterParams, SpatialFilterParams, SpeckleFilterParams, TemporalFilterParams> FilterParams;

class ImageFiltersConfig : public Buffer {
   public:
    virtual ~ImageFiltersConfig();
    void serialize(std::vector<std::uint8_t>& metadata, DatatypeEnum& datatype) const override;

    DatatypeEnum getDatatype() const override {
        return DatatypeEnum::ImageFiltersConfig;
    }

    /**
     * Insert filter parameters describing how a filter at index index should be updated
     * @param index Index of the filter to be inserted
     * @param params Parameters of the filter to be inserted
     */
    ImageFiltersConfig& updateFilterAtIndex(std::int32_t index, FilterParams params);

    /**
     * Insert filter parameters describing how a new filter should be inserted
     * @param params Parameters of the filter to be inserted
     */
    ImageFiltersConfig& insertFilter(FilterParams params);

    /**
     * Index of the filter to be applied
     */
    std::vector<std::int32_t> filterIndices = {};

    /**
     * Parameters of the filter to be applied
     */
    std::vector<FilterParams> filterParams = {};

    /**
     * Set preset mode for ImageFiltersConfig.
     * @param presetMode Preset mode for ImageFiltersConfig.
     */
    void setProfilePreset(ImageFiltersPresetMode presetMode);

    DEPTHAI_SERIALIZE(ImageFiltersConfig, filterIndices, filterParams);
};

class ToFDepthConfidenceFilterConfig : public Buffer {
   public:
    virtual ~ToFDepthConfidenceFilterConfig();
    void serialize(std::vector<std::uint8_t>& metadata, DatatypeEnum& datatype) const override;

    DatatypeEnum getDatatype() const override {
        return DatatypeEnum::ToFDepthConfidenceFilterConfig;
    }

    /**
     * Threshold for the confidence filter
     */
    float confidenceThreshold = 0.1f;

    /**
     * Set preset mode for ImageFiltersPresetMode.
     * @param presetMode Preset mode for ImageFiltersPresetMode.
     */
    void setProfilePreset(ImageFiltersPresetMode presetMode);

    DEPTHAI_SERIALIZE(ToFDepthConfidenceFilterConfig, confidenceThreshold);
};

}  // namespace dai