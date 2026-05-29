#pragma once

#include "depthai/common/ImgTransformations.hpp"
#include "depthai/common/optional.hpp"
#include "depthai/pipeline/datatype/Buffer.hpp"
#include "depthai/pipeline/datatype/DatatypeEnum.hpp"
#include "depthai/utility/Serialization.hpp"

namespace dai {

/**
 * Interface for messages that carry image transformation metadata and can be remapped
 * to another image transformation.
 */
class Transformable {
   protected:
    /**
     * Internal function to perform the transformation, to be implemented by derived classes.
     */
    virtual void transformToInternal(const ImgTransformation& target) = 0;

   public:
    virtual ~Transformable();
    std::optional<ImgTransformation> transformation;

    /**
     * Returns the current transformation if set, else std::nullopt.
     */
    std::optional<ImgTransformation> getTransformation() const;

    /**
     * Sets the current transformation.
     */
    void setTransformation(const ImgTransformation& transformation);

    DEPTHAI_SERIALIZE(Transformable, transformation);
};

template <class Derived>
class TransformableCRTP : public Transformable {
   public:
    /*
     * Returns a new message that has been transformed into the target image transformation. The source transformation is taken from the message itself, and an
     * error is thrown if it is not set.
     *
     * The Derived class is expected to override the transformToInternal function, which performs the actual transformation of the message data.
     */
    Derived transformTo(const ImgTransformation& target) const {
        Derived out(static_cast<const Derived&>(*this));
        out.transformToInternal(target);
        return out;
    }
};

/**
 * Base message type for custom transformable messages.
 *
 * This class combines Buffer storage with the Transformable interface and provides a
 * virtual transformTo() entry point for host-side polymorphic dispatch. Python users
 * should inherit from TransformableBuffer, not Transformable, when implementing custom
 * messages that need transformation support.
 */
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

    /**
     * Returns a transformed copy of this message.
     *
     * Python subclasses of TransformableBuffer should override this method to implement
     * custom transformation logic. Generic callers use this entry point when they need to
     * transform a custom message without knowing its concrete type. The base implementation
     * throws, because it cannot preserve Python subclass state automatically.
     *
     * @param target Target image transformation.
     */
    virtual std::shared_ptr<TransformableBuffer> transformTo(const ImgTransformation& target) const;

    void serialize(std::vector<std::uint8_t>& metadata, DatatypeEnum& datatype) const override;

    DEPTHAI_SERIALIZE(TransformableBuffer, sequenceNum, ts, tsDevice, Transformable::transformation);
};

}  // namespace dai
