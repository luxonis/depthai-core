#pragma once

#include "depthai/common/ImgTransformations.hpp"
#include "depthai/common/optional.hpp"
#include "depthai/pipeline/datatype/Buffer.hpp"
#include "depthai/pipeline/datatype/DatatypeEnum.hpp"
#include "depthai/utility/Serialization.hpp"

namespace dai {

class Transformable {
   protected:
    virtual void transformToInternal(const ImgTransformation& target) = 0;

   public:
    virtual ~Transformable();
    std::optional<ImgTransformation> transformation;

    std::optional<ImgTransformation> getTransformation() const;

    void setTransformation(const ImgTransformation& transformation);

    DEPTHAI_SERIALIZE(Transformable, transformation);
};

template <class Derived>
class TransformableCRTP : public Transformable {
   public:
    Derived transformTo(const ImgTransformation& target) const {
        Derived out(static_cast<const Derived&>(*this));
        out.transformToInternal(target);
        return out;
    }
};

class TransformableBuffer : public Buffer, public Transformable {
   protected:
    void transformToInternal(const ImgTransformation& target) override;

   public:
    using Buffer::sequenceNum;
    using Buffer::ts;
    using Buffer::tsDevice;
    using Transformable::getTransformation;
    using Transformable::setTransformation;
    using Transformable::transformation;

    DatatypeEnum getDatatype() const override {
        return DatatypeEnum::Transformable;
    }

    TransformableBuffer() = default;
    virtual ~TransformableBuffer();

    virtual std::shared_ptr<TransformableBuffer> transformTo(const ImgTransformation& target) const;

    void serialize(std::vector<std::uint8_t>& metadata, DatatypeEnum& datatype) const override;

    DEPTHAI_SERIALIZE(TransformableBuffer, sequenceNum, ts, tsDevice, Transformable::transformation);
};

}  // namespace dai
