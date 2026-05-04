#pragma once

#include "depthai/common/ImgTransformations.hpp"
#include "depthai/common/optional.hpp"
#include "depthai/pipeline/datatype/Buffer.hpp"
#include "depthai/utility/Serialization.hpp"

namespace dai {

class TransformableBuffer : public Buffer {
   protected:
    virtual void transformToInternal(const ImgTransformation& target) = 0;

   public:
    virtual ~TransformableBuffer();
    using Buffer::Buffer;
    std::optional<ImgTransformation> transformation;

    std::optional<ImgTransformation> getTransformation() const;

    void setTransformation(const ImgTransformation& transformation);

    DEPTHAI_SERIALIZE(TransformableBuffer, Buffer::ts, Buffer::tsDevice, Buffer::sequenceNum, transformation);
};

}  // namespace dai