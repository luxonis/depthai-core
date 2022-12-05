#pragma once

#include <unordered_map>
#include <vector>

#include "depthai/xlink/XLinkStream.hpp"

#include "depthai/utility/Memory.hpp"

#include "depthai-shared/datatype/RawBuffer.hpp"

namespace dai {

/// Abstract message
class ADatatype {
   protected:
    friend class DataInputQueue;
    friend class StreamMessageParser;
    std::shared_ptr<RawBuffer> raw;

   public:

    struct Serialized {
        std::shared_ptr<Memory> data;
        std::shared_ptr<RawBuffer> metadata;
    };

    explicit ADatatype(std::shared_ptr<RawBuffer> r) : raw(std::move(r)) {}
    virtual ~ADatatype() = default;
    // TBD
    virtual Serialized serialize() const = 0;
    std::shared_ptr<RawBuffer> getRaw() const {
        return raw;
    }

    std::shared_ptr<Memory> data;
};

}  // namespace dai
