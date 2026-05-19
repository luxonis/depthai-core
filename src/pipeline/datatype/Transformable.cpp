#include "depthai/pipeline/datatype/Transformable.hpp"

#include "depthai/common/ImgTransformations.hpp"

namespace dai {

Transformable::~Transformable() = default;

TransformableBuffer::~TransformableBuffer() = default;

std::optional<ImgTransformation> Transformable::getTransformation() const {
    return transformation;
}

void Transformable::setTransformation(const ImgTransformation& transformation) {
    this->transformation = transformation;
}

void Transformable::transformTo(const ImgTransformation& target) {
    transformToInternal(target);
}

}  // namespace dai
