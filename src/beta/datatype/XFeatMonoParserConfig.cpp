#include "depthai/beta/datatype/XFeatMonoParserConfig.hpp"

#include "utility/ErrorMacros.hpp"

namespace dai {
namespace beta {

XFeatMonoParserConfig::~XFeatMonoParserConfig() = default;

void XFeatMonoParserConfig::serialize(std::vector<std::uint8_t>& metadata, DatatypeEnum& datatype) const {
    metadata = utility::serialize(*this);
    datatype = getDatatype();
}

bool XFeatMonoParserConfig::validate() const {
    return maxKeypoints > 0;
}

void XFeatMonoParserConfig::setMaxKeypoints(int value) {
    auto candidate = *this;
    candidate.maxKeypoints = value;
    DAI_CHECK(candidate.validate(), "Maximum keypoints must be positive.");
    maxKeypoints = value;
}

int XFeatMonoParserConfig::getMaxKeypoints() const {
    return maxKeypoints;
}

}  // namespace beta
}  // namespace dai
