#include "depthai/pipeline/datatype/TransformableBuffer.hpp"

#include "depthai/common/ImgTransformations.hpp"

namespace dai {

std::optional<ImgTransformation> TransformableBuffer::getTransformation() const {
    return transformation;
}

void TransformableBuffer::setTransformation(const ImgTransformation& transformation) {
    this->transformation = transformation;
}

}  // namespace dai
