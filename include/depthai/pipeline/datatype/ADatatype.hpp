#pragma once

#include <unordered_map>
#include <vector>

#include "depthai-shared/datatype/RawBuffer.hpp"

namespace dai {

class ADatatype {
   protected:
    friend class DataInputQueue;
    virtual std::shared_ptr<dai::RawBuffer> serialize() const = 0;
    std::shared_ptr<RawBuffer> raw;

   public:
    ADatatype(std::shared_ptr<RawBuffer> r) : raw(r) {}
    std::shared_ptr<RawBuffer> getRaw() {
        return raw;
    }
};

}  // namespace dai
