#include "depthai/pipeline/datatype/Transformable.hpp"

#include <stdexcept>

#include "depthai/common/ImgTransformations.hpp"

namespace dai {

TransformableBuffer::~TransformableBuffer() = default;

void TransformableBuffer::transformToInternal(const ImgTransformation&) {
    // No-op placeholder for the abstract Transformable interface.
}

std::shared_ptr<TransformableBuffer> TransformableBuffer::transformTo(const ImgTransformation& target) const {
    static_cast<void>(target);
    throw std::runtime_error(
        "TransformableBuffer::transformTo() is not implemented for this message. "
        "Python subclasses of dai.TransformableBuffer must override transformTo(target) "
        "and return a transformed instance.");
}

void TransformableBuffer::serialize(std::vector<std::uint8_t>& metadata, DatatypeEnum& datatype) const {
    metadata = utility::serialize(*this);
    datatype = this->getDatatype();
}

Transformable::~Transformable() = default;

std::optional<ImgTransformation> Transformable::getTransformation() const {
    return transformation;
}

void Transformable::setTransformation(const ImgTransformation& transformation) {
    this->transformation = transformation;
}

}  // namespace dai
