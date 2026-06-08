#pragma once

#include "depthai/pipeline/datatype/Buffer.hpp"

namespace dai {

class MissingDataMessage : public Buffer {
   public:
    MissingDataMessage() = default;
    ~MissingDataMessage() override;

    void serialize(std::vector<std::uint8_t>& metadata, DatatypeEnum& datatype) const override {
        metadata = utility::serialize(*this);
        datatype = DatatypeEnum::MissingDataMessage;
    }

    DatatypeEnum getDatatype() const override {
        return DatatypeEnum::MissingDataMessage;
    }

    DEPTHAI_SERIALIZE(MissingDataMessage, Buffer::sequenceNum, Buffer::ts, Buffer::tsDevice);
};

}  // namespace dai
