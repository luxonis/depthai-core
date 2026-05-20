#include "depthai/pipeline/datatype/Transformable.hpp"

#include "depthai/common/ImgTransformations.hpp"

namespace dai {

TransformableBuffer::~TransformableBuffer() = default;

void TransformableBuffer::transformToInternal(const ImgTransformation&) {
    // Default implementation is a no-op so custom Python subclasses can
    // opt into copy-return semantics by overriding transformTo().
}

std::shared_ptr<TransformableBuffer> TransformableBuffer::transformTo(const ImgTransformation& target) const {
    auto out = std::make_shared<TransformableBuffer>(*this);
    out->transformToInternal(target);
    return out;
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
