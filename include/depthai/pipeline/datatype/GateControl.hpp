#pragma once

#include "depthai/pipeline/datatype/Buffer.hpp"

namespace dai {

class GateControl : public Buffer {
   public:
    GateControl() = default;

    ~GateControl() override;

    bool value = true;

    void start() {
        value = true;
    }

    void stop() {
        value = false;
    }

    void serialize(std::vector<std::uint8_t>& metadata, DatatypeEnum& datatype) const override {
        metadata = utility::serialize(*this);
        datatype = this->getDatatype();
    }

    DatatypeEnum getDatatype() const override {
        return DatatypeEnum::GateControl;
    }

    DEPTHAI_SERIALIZE(GateControl, value);
};

}  // namespace dai
