#include "depthai/beta/datatype/ImgDetectionsFilterConfig.hpp"

namespace dai {
namespace beta {

ImgDetectionsFilterConfig::~ImgDetectionsFilterConfig() = default;

void ImgDetectionsFilterConfig::serialize(std::vector<std::uint8_t>& metadata, DatatypeEnum& datatype) const {
    metadata = utility::serialize(*this);
    datatype = getDatatype();
}

bool ImgDetectionsFilterConfig::isNoOp() const {
    return !labelsToKeep.has_value() && !labelsToReject.has_value() && !confidenceThreshold.has_value() && !minArea.has_value() && nmsDisabled
           && sortingDisabled && !firstK.has_value();
}

}  // namespace beta
}  // namespace dai
