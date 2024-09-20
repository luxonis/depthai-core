#pragma once
#include <cstdint>
#include <vector>

#include "depthai/pipeline/datatype/Buffer.hpp"
#include "depthai/pipeline/datatype/DatatypeEnum.hpp"

namespace dai {

/// ImageAlignConfig configuration structure
class ImageAlignConfig : public Buffer {
   public:
    /**
     * Optional static depth plane to align to, in depth units, by default millimeters
     */
    uint16_t staticDepthPlane = 0;

    ImageAlignConfig() = default;
    virtual ~ImageAlignConfig() = default;

    void serialize(std::vector<std::uint8_t>& metadata, DatatypeEnum& datatype) const override {
        metadata = utility::serialize(*this);
        datatype = DatatypeEnum::ImageAlignConfig;
    };

    DEPTHAI_SERIALIZE(ImageAlignConfig, staticDepthPlane);
};

}  // namespace dai