#include "depthai/pipeline/datatype/TransformableBuffer.hpp"

#include "depthai/common/ImgTransformations.hpp"

namespace dai {

TransformableBuffer::~TransformableBuffer() = default;

std::optional<ImgTransformation> TransformableBuffer::getTransformation() const {
    return transformation;
}

void TransformableBuffer::setTransformation(const ImgTransformation& transformation) {
    this->transformation = transformation;
}

std::shared_ptr<TransformableBuffer> TransformableBuffer::cloneAndTransformTo(const ImgTransformation& target) const {
    auto out = clone();
    if(!out) {
        throw std::logic_error("TransformableBuffer::clone() returned nullptr");
    }

    out->transformToInternal(target);
    return out;
}

}  // namespace dai
