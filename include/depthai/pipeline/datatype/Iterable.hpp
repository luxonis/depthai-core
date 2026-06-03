#pragma once

#include <cstddef>

#include "depthai/pipeline/datatype/Buffer.hpp"
#include "depthai/pipeline/datatype/Transformable.hpp"

namespace dai {

/**
 * Interface for messages that contain iterable sub-items.
 */
// template <class Item>
class Iterable {
   public:
    // if we wanted to even iterate over items without issues:
    //    std::vector<Item>& items;

    virtual ~Iterable() = default;

    /**
     * Returns the number of iterable sub-items contained in the message.
     */
    virtual std::size_t getSize() const = 0;
};

/**
 * Base message type for custom iterable messages.
 *
 * Python users should inherit from IterableBuffer, not Buffer, when implementing
 * custom messages that expose an iterable item count to host-side logic.
 */
class IterableBuffer : public Buffer, public Iterable {
   public:
    using Buffer::sequenceNum;
    using Buffer::ts;
    using Buffer::tsDevice;

    IterableBuffer() = default;
    ~IterableBuffer() override = default;
};

/**
 * Base message type for custom iterable messages that are also transformable.
 *
 * Python users should inherit from IterableTransformableBuffer when implementing
 * custom messages that need both transformTo() and getSize() overrides.
 */
class IterableTransformableBuffer : public TransformableBuffer, public Iterable {
   public:
    using Buffer::sequenceNum;
    using Buffer::ts;
    using Buffer::tsDevice;
    using Transformable::getTransformation;
    using Transformable::setTransformation;
    using Transformable::transformation;

    // DATATYPE here is Transformable (from buffer) but not iterable... might need to change to IterableTransformableBuffer

    IterableTransformableBuffer() = default;
    ~IterableTransformableBuffer() override = default;
};

}  // namespace dai
