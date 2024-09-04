#pragma once
#include "depthai/pipeline/datatype/Buffer.hpp"
#include "depthai/utility/Serialization.hpp"
namespace dai {

/**
 * ThermalConfig message. Currently unused.
 */
class ThermalConfig : public Buffer {
   public:
    bool autoFFC = true;
    /**
     * Construct ThermalConfig message.
     */
    ThermalConfig() = default;
    virtual ~ThermalConfig() = default;

    void serialize(std::vector<std::uint8_t>& metadata, DatatypeEnum& datatype) const override {
        metadata = utility::serialize(*this);
        datatype = DatatypeEnum::ThermalConfig;
    };

    DEPTHAI_SERIALIZE(ThermalConfig, autoFFC);
};

}  // namespace dai
