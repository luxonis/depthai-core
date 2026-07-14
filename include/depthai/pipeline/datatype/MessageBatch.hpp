#pragma once

#include <cstdint>
#include <memory>
#include <vector>

#include "depthai/pipeline/datatype/Buffer.hpp"

namespace dai {

/**
 * Message that carries an ordered collection of Buffer-derived messages.
 *
 * Missing entries are represented as nullptr to preserve iterable item positions.
 */
class MessageBatch : public Buffer {
   public:
    MessageBatch() = default;
    explicit MessageBatch(std::vector<std::shared_ptr<Buffer>> buffers);
    ~MessageBatch() override;

    void serialize(std::vector<std::uint8_t>& metadata, DatatypeEnum& datatype) const override;

    DatatypeEnum getDatatype() const override {
        return DatatypeEnum::MessageBatch;
    }

    void setBuffers(const std::vector<std::shared_ptr<Buffer>>& buffers);
    void setBuffers(std::vector<std::shared_ptr<Buffer>>&& buffers);
    const std::vector<std::shared_ptr<Buffer>>& getBuffers() const;

    void push_back(const std::shared_ptr<Buffer>& buffer);
    std::shared_ptr<Buffer> at(std::size_t index) const;
    std::shared_ptr<Buffer> operator[](std::size_t index) const;
    std::size_t size() const;
    bool empty() const;

    std::vector<std::shared_ptr<Buffer>>::iterator begin();
    std::vector<std::shared_ptr<Buffer>>::iterator end();
    std::vector<std::shared_ptr<Buffer>>::const_iterator begin() const;
    std::vector<std::shared_ptr<Buffer>>::const_iterator end() const;

    void restoreBufferSlots();

   public:
    std::vector<std::shared_ptr<Buffer>> buffers;

    // Internal serialized representation used for transport.
    std::vector<std::uint8_t> itemPresent;

    DEPTHAI_SERIALIZE(MessageBatch, itemPresent, Buffer::ts, Buffer::tsDevice, Buffer::tsSystem, Buffer::sequenceNum);
};

}  // namespace dai
