#include "depthai/pipeline/datatype/MessageBatch.hpp"

#include <utility>

namespace dai {

MessageBatch::MessageBatch(std::vector<std::shared_ptr<Buffer>> buffers) : buffers(std::move(buffers)) {}

MessageBatch::~MessageBatch() = default;

void MessageBatch::serialize(std::vector<std::uint8_t>& metadata, DatatypeEnum& datatype) const {
    MessageBatch copy = *this;
    copy.itemPresent.clear();
    copy.itemPresent.reserve(copy.buffers.size());

    for(const auto& buffer : copy.buffers) {
        copy.itemPresent.push_back(buffer ? 1U : 0U);
    }

    metadata = utility::serialize(copy);
    datatype = DatatypeEnum::MessageBatch;
}

void MessageBatch::setBuffers(const std::vector<std::shared_ptr<Buffer>>& buffers) {
    this->buffers = buffers;
}

void MessageBatch::setBuffers(std::vector<std::shared_ptr<Buffer>>&& buffers) {
    this->buffers = std::move(buffers);
}

const std::vector<std::shared_ptr<Buffer>>& MessageBatch::getBuffers() const {
    return buffers;
}

void MessageBatch::push_back(const std::shared_ptr<Buffer>& buffer) {
    buffers.push_back(buffer);
}

std::shared_ptr<Buffer> MessageBatch::at(std::size_t index) const {
    return buffers.at(index);
}

std::shared_ptr<Buffer> MessageBatch::operator[](std::size_t index) const {
    return buffers[index];
}

std::size_t MessageBatch::size() const {
    return buffers.size();
}

bool MessageBatch::empty() const {
    return buffers.empty();
}

std::vector<std::shared_ptr<Buffer>>::iterator MessageBatch::begin() {
    return buffers.begin();
}

std::vector<std::shared_ptr<Buffer>>::iterator MessageBatch::end() {
    return buffers.end();
}

std::vector<std::shared_ptr<Buffer>>::const_iterator MessageBatch::begin() const {
    return buffers.begin();
}

std::vector<std::shared_ptr<Buffer>>::const_iterator MessageBatch::end() const {
    return buffers.end();
}

void MessageBatch::restoreBufferSlots() {
    buffers.assign(itemPresent.size(), nullptr);
}

}  // namespace dai
