#pragma once

#include <cstdint>
#include <memory>
#include <vector>

#include "depthai/pipeline/datatype/Buffer.hpp"

namespace dai {

class BatchItem : public Buffer {
   public:
    BatchItem() = default;
    BatchItem(std::shared_ptr<ADatatype> payload, std::uint32_t batchSize, std::uint32_t batchIndex, std::uint32_t itemIndex);
    ~BatchItem() override;

    void serialize(std::vector<std::uint8_t>& metadata, DatatypeEnum& datatype) const override;

    DatatypeEnum getDatatype() const override {
        return DatatypeEnum::BatchItem;
    }

    void setPayload(std::shared_ptr<ADatatype> payload);

    std::shared_ptr<ADatatype> getPayload() const;

    template <typename T>
    std::shared_ptr<T> getPayload() const {
        return std::dynamic_pointer_cast<T>(payload);
    }

    std::uint32_t batchSize = 0;
    std::uint32_t batchIndex = 0;
    std::uint32_t itemIndex = 0;

    DatatypeEnum payloadDatatype = DatatypeEnum::ADatatype;
    std::vector<std::uint8_t> payloadMetadata;

    DEPTHAI_SERIALIZE(
        BatchItem, Buffer::sequenceNum, Buffer::ts, Buffer::tsDevice, Buffer::tsSystem, batchSize, batchIndex, itemIndex, payloadDatatype, payloadMetadata);

   private:
    std::shared_ptr<ADatatype> payload;
};

}  // namespace dai
