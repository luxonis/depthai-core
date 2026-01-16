#pragma once

#include "depthai/pipeline/datatype/Buffer.hpp"

namespace dai {

class GateControl : public Buffer {
   public:
    bool open = true;

    int numMessages = -1;

    static std::shared_ptr<GateControl> openGate(int numMessages) {
        return std::make_shared<GateControl>(true, numMessages);
    }

    static std::shared_ptr<GateControl> openGate() {
        return std::make_shared<GateControl>(true, -1);
    }

    static std::shared_ptr<GateControl> closeGate() {
        return std::make_shared<GateControl>(false, -1);
    }

    GateControl() = default;

    GateControl(bool open, int numMessages) : open(open), numMessages(numMessages) {};

    ~GateControl() override;

    void serialize(std::vector<std::uint8_t>& metadata, DatatypeEnum& datatype) const override {
        metadata = utility::serialize(*this);
        datatype = this->getDatatype();
    }

    DatatypeEnum getDatatype() const override {
        return DatatypeEnum::GateControl;
    }

    DEPTHAI_SERIALIZE(GateControl, open, numMessages);
};

}  // namespace dai
