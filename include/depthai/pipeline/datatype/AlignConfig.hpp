#pragma once
#include <cstdint>
#include <vector>

#include "depthai/pipeline/datatype/Buffer.hpp"
#include "depthai/pipeline/datatype/DatatypeEnum.hpp"

namespace dai {

/// AlignConfig configuration structure
class AlignConfig : public Buffer {
   public:
    /**
     * Optional static depth plane to align to, in depth units, by default millimeters
     */
    uint16_t staticDepthPlane = 0;

    AlignConfig() = default;
    virtual ~AlignConfig();

    void serialize(std::vector<std::uint8_t>& metadata, DatatypeEnum& datatype) const override;

    DatatypeEnum getDatatype() const override {
        return DatatypeEnum::AlignConfig;
    }

    DEPTHAI_SERIALIZE(AlignConfig, staticDepthPlane);
};

}  // namespace dai
