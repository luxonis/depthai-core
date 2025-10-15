#include "depthai/pipeline/datatype/ImageFiltersConfig.hpp"

#include "utility/ErrorMacros.hpp"

namespace dai {

ImageFiltersConfig::~ImageFiltersConfig() = default;

void ImageFiltersConfig::serialize(std::vector<std::uint8_t>& metadata, DatatypeEnum& datatype) const {
    metadata = utility::serialize(*this);
    datatype = DatatypeEnum::ImageFiltersConfig;
}

ToFDepthConfidenceFilterConfig::~ToFDepthConfidenceFilterConfig() = default;

void ToFDepthConfidenceFilterConfig::serialize(std::vector<std::uint8_t>& metadata, DatatypeEnum& datatype) const {
    metadata = utility::serialize(*this);
    datatype = DatatypeEnum::ToFDepthConfidenceFilterConfig;
}

ImageFiltersConfig& ImageFiltersConfig::updateFilterAtIndex(std::int32_t index, FilterParams params) {
    DAI_CHECK_V(this->filterIndices.size() == this->filterParams.size(),
                "ImageFiltersConfig can either be used to create a new filter pipeline or update an existing one, not both");
    this->filterIndices.push_back(index);
    this->filterParams.push_back(params);
    return *this;
}

ImageFiltersConfig& ImageFiltersConfig::insertFilter(FilterParams params) {
    DAI_CHECK_V(filterIndices.size() == 0, "ImageFiltersConfig can either be used to create a new filter pipeline or update an existing one, not both");
    this->filterParams.push_back(params);
    return *this;
}

void ImageFiltersConfig::setProfilePreset(ImageFiltersPresetMode presetMode) {
    switch(presetMode) {
        case ImageFiltersPresetMode::TOF_LOW_RANGE: {
            std::vector<FilterParams> params;

            TemporalFilterParams temporalFilterParams;
            temporalFilterParams.enable = true;
            temporalFilterParams.persistencyMode = TemporalFilterParams::PersistencyMode::VALID_1_IN_LAST_8;
            temporalFilterParams.alpha = 0.2f;
            temporalFilterParams.delta = 60;

            SpeckleFilterParams speckleFilterParams;
            speckleFilterParams.enable = true;
            speckleFilterParams.speckleRange = 14;
            speckleFilterParams.differenceThreshold = 11;

            SpatialFilterParams spatialFilterParams;
            spatialFilterParams.enable = true;
            spatialFilterParams.alpha = 0.5f;
            spatialFilterParams.delta = 20;
            spatialFilterParams.numIterations = 2;
            spatialFilterParams.holeFillingRadius = 0;

            MedianFilterParams medianFilterParams = MedianFilterParams::MEDIAN_OFF;

            params.push_back(temporalFilterParams);
            params.push_back(speckleFilterParams);
            params.push_back(spatialFilterParams);
            params.push_back(medianFilterParams);

            this->filterParams = params;
            this->filterIndices = {};
        } break;

        case ImageFiltersPresetMode::TOF_MID_RANGE: {
            std::vector<FilterParams> params;

            TemporalFilterParams temporalFilterParams;
            temporalFilterParams.enable = true;
            temporalFilterParams.persistencyMode = TemporalFilterParams::PersistencyMode::VALID_1_IN_LAST_5;
            temporalFilterParams.alpha = 0.1f;
            temporalFilterParams.delta = 40;

            SpeckleFilterParams speckleFilterParams;
            speckleFilterParams.enable = true;
            speckleFilterParams.speckleRange = 6;
            speckleFilterParams.differenceThreshold = 130;

            SpatialFilterParams spatialFilterParams;
            spatialFilterParams.enable = true;
            spatialFilterParams.alpha = 0.5f;
            spatialFilterParams.delta = 20;
            spatialFilterParams.numIterations = 1;
            spatialFilterParams.holeFillingRadius = 2;

            MedianFilterParams medianFilterParams = MedianFilterParams::MEDIAN_OFF;

            params.push_back(temporalFilterParams);
            params.push_back(speckleFilterParams);
            params.push_back(spatialFilterParams);
            params.push_back(medianFilterParams);

            this->filterParams = params;
            this->filterIndices = {};
        } break;

        case ImageFiltersPresetMode::TOF_HIGH_RANGE: {
            std::vector<FilterParams> params;

            TemporalFilterParams temporalFilterParams;
            temporalFilterParams.enable = true;
            temporalFilterParams.persistencyMode = TemporalFilterParams::PersistencyMode::PERSISTENCY_INDEFINITELY;
            temporalFilterParams.alpha = 0.2f;
            temporalFilterParams.delta = 60;

            SpeckleFilterParams speckleFilterParams;
            speckleFilterParams.enable = true;
            speckleFilterParams.speckleRange = 4;
            speckleFilterParams.differenceThreshold = 130;

            SpatialFilterParams spatialFilterParams;
            spatialFilterParams.enable = true;
            spatialFilterParams.alpha = 0.5f;
            spatialFilterParams.delta = 50;
            spatialFilterParams.numIterations = 2;
            spatialFilterParams.holeFillingRadius = 0;

            MedianFilterParams medianFilterParams = MedianFilterParams::KERNEL_5x5;

            params.push_back(temporalFilterParams);
            params.push_back(speckleFilterParams);
            params.push_back(spatialFilterParams);
            params.push_back(medianFilterParams);

            this->filterParams = params;
            this->filterIndices = {};
        } break;
    }
}

void ToFDepthConfidenceFilterConfig::setProfilePreset(ImageFiltersPresetMode presetMode) {
    switch(presetMode) {
        case ImageFiltersPresetMode::TOF_LOW_RANGE: {
            this->confidenceThreshold = 0.1f;
        } break;
        case ImageFiltersPresetMode::TOF_MID_RANGE: {
            this->confidenceThreshold = 0.2f;
        } break;
        case ImageFiltersPresetMode::TOF_HIGH_RANGE: {
            this->confidenceThreshold = 0.3f;
        } break;
    }
}

}  // namespace dai