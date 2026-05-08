#pragma once

#include "depthai/common/ImgTransformations.hpp"
#include "depthai/common/optional.hpp"
#include "depthai/pipeline/datatype/Buffer.hpp"
#include "depthai/utility/Serialization.hpp"

namespace dai {

class TransformableBuffer : public Buffer {
   protected:
    virtual void transformToInternal(const ImgTransformation& target) = 0;
    virtual std::shared_ptr<TransformableBuffer> clone() const = 0;

   public:
    virtual ~TransformableBuffer();
    using Buffer::Buffer;
    std::optional<ImgTransformation> transformation;

    std::optional<ImgTransformation> getTransformation() const;

    void setTransformation(const ImgTransformation& transformation);

    std::shared_ptr<TransformableBuffer> cloneAndTransformTo(const ImgTransformation& target) const;

    DatatypeEnum getDatatype() const override {
        return DatatypeEnum::TransformableBuffer;
    }

    DEPTHAI_SERIALIZE(TransformableBuffer, Buffer::ts, Buffer::tsDevice, Buffer::sequenceNum, transformation);
};

}  // namespace dai