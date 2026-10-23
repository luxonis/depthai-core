#include "depthai/pipeline/datatype/ImgDetectionsFilterConfig.hpp"

namespace dai {
ImgDetectionsFilterConfig::~ImgDetectionsFilterConfig() = default;

void ImgDetectionsFilterConfig::serialize(std::vector<std::uint8_t>& metadata, DatatypeEnum& datatype) const {
    metadata = utility::serialize(*this);
    datatype = getDatatype();
}

ImgDetectionsFilterConfig& ImgDetectionsFilterConfig::setConfidenceRange(float min, float max) {
    minConfidence = min;
    maxConfidence = max;
    return *this;
}
ImgDetectionsFilterConfig& ImgDetectionsFilterConfig::setSizeRange(float minArea, float maxArea) {
    this->minArea = minArea;
    this->maxArea = maxArea;
    return *this;
}
ImgDetectionsFilterConfig& ImgDetectionsFilterConfig::setWidthRange(float min, float max) {
    minWidth = min;
    maxWidth = max;
    return *this;
}
ImgDetectionsFilterConfig& ImgDetectionsFilterConfig::setHeightRange(float min, float max) {
    minHeight = min;
    maxHeight = max;
    return *this;
}
bool ImgDetectionsFilterConfig::validate() const {
    return minConfidence < maxConfidence && minArea < maxArea && minWidth < maxWidth && minHeight < maxHeight;
}
bool ImgDetectionsFilterConfig::hasGeometryFilters() const {
    const auto unlimited = std::numeric_limits<float>::max();
    return minArea != 0 || maxArea != unlimited || minWidth != 0 || maxWidth != unlimited || minHeight != 0 || maxHeight != unlimited
           || regionOfInterest.has_value();
}
}  // namespace dai
