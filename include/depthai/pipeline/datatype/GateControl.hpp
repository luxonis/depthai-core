#pragma once

#include "depthai/pipeline/datatype/Buffer.hpp"

namespace dai {

class GateControl : public Buffer {
   public:
    bool open = true;

    int numMessages = -1;

    int fps = -1;

    static std::shared_ptr<GateControl> openGate(int numMessages, int fps = -1) {
        return std::make_shared<GateControl>(true, numMessages, fps);
    }

    static std::shared_ptr<GateControl> openGate() {
        return std::make_shared<GateControl>(true, -1, -1);
    }

    static std::shared_ptr<GateControl> closeGate() {
        return std::make_shared<GateControl>(false, -1, -1);
    }

    GateControl() = default;

    GateControl(bool open, int numMessages, int fps) : open(open), numMessages(numMessages), fps(fps) {};

    ~GateControl() override;

    void serialize(std::vector<std::uint8_t>& metadata, DatatypeEnum& datatype) const override {
        metadata = utility::serialize(*this);
        datatype = this->getDatatype();
    }

    DatatypeEnum getDatatype() const override {
        return DatatypeEnum::GateControl;
    }

    DEPTHAI_SERIALIZE(GateControl, open, numMessages, fps);
};

}  // namespace dai
