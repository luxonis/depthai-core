#include "depthai/beta/datatype/XFeatStereoParserConfig.hpp"

#include "utility/ErrorMacros.hpp"

namespace dai {
namespace beta {

XFeatStereoParserConfig::~XFeatStereoParserConfig() = default;

void XFeatStereoParserConfig::serialize(std::vector<std::uint8_t>& metadata, DatatypeEnum& datatype) const {
    metadata = utility::serialize(*this);
    datatype = getDatatype();
}

bool XFeatStereoParserConfig::validate() const {
    return maxKeypoints > 0;
}

void XFeatStereoParserConfig::setMaxKeypoints(int value) {
    auto candidate = *this;
    candidate.maxKeypoints = value;
    DAI_CHECK(candidate.validate(), "Maximum keypoints must be positive.");
    maxKeypoints = value;
}

int XFeatStereoParserConfig::getMaxKeypoints() const {
    return maxKeypoints;
}

}  // namespace beta
}  // namespace dai
