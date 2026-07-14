#include "depthai/pipeline/datatype/BatchItem.hpp"

#include <utility>

#include "depthai/utility/VectorMemory.hpp"

namespace dai {

BatchItem::BatchItem(std::shared_ptr<ADatatype> payload, std::uint32_t batchSize, std::uint32_t batchIndex, std::uint32_t itemIndex)
    : batchSize(batchSize), batchIndex(batchIndex), itemIndex(itemIndex) {
    setPayload(std::move(payload));
}

BatchItem::~BatchItem() = default;

void BatchItem::serialize(std::vector<std::uint8_t>& metadata, DatatypeEnum& datatype) const {
    BatchItem copy = *this;
    copy.payloadMetadata.clear();
    copy.payloadDatatype = DatatypeEnum::ADatatype;
    if(copy.payload) {
        copy.data = copy.payload->data;
        copy.payload->serialize(copy.payloadMetadata, copy.payloadDatatype);
    }
    metadata = utility::serialize(copy);
    datatype = DatatypeEnum::BatchItem;
}

void BatchItem::setPayload(std::shared_ptr<ADatatype> payload) {
    this->payload = std::move(payload);
    if(this->payload) {
        data = this->payload->data;
        setBufferMetadataFrom(std::dynamic_pointer_cast<Buffer>(this->payload));
    } else {
        data = std::make_shared<VectorMemory>(std::vector<std::uint8_t>{});
    }
}

std::shared_ptr<ADatatype> BatchItem::getPayload() const {
    return payload;
}

}  // namespace dai
