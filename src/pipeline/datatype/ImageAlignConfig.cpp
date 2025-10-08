#include "depthai/pipeline/datatype/ImageAlignConfig.hpp"

namespace dai {

ImageAlignConfig::~ImageAlignConfig() = default;

void ImageAlignConfig::serialize(std::vector<std::uint8_t>& metadata, DatatypeEnum& datatype) const {
    metadata = utility::serialize(*this);
    datatype = DatatypeEnum::ImageAlignConfig;
}

}  // namespace dai