#include "depthai/pipeline/datatype/ImageFiltersConfig.hpp"

#include "utility/ErrorMacros.hpp"

namespace dai {
ImageFiltersConfig& ImageFiltersConfig::updateFilterAtIndex(std::int32_t filterIndex, FilterParams filterParams) {
    DAI_CHECK_V(this->filterIndices.size() == this->filterParams.size(),
                "ImageFiltersConfig can either be used to create a new filter pipeline or update an existing one, not both");
    this->filterIndices.push_back(filterIndex);
    this->filterParams.push_back(filterParams);
    return *this;
}

ImageFiltersConfig& ImageFiltersConfig::insertFilter(FilterParams filterParams) {
    DAI_CHECK_V(filterIndices.size() == 0, "ImageFiltersConfig can either be used to create a new filter pipeline or update an existing one, not both");
    this->filterParams.push_back(filterParams);
    return *this;
}

}  // namespace dai