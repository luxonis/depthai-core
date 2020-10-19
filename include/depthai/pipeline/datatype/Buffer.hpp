#pragma once

#include <unordered_map>
#include <vector>

#include "depthai-shared/datatype/RawBuffer.hpp"
#include "depthai/pipeline/datatype/ADatatype.hpp"

namespace dai {
// protected inheritance, so serialize isn't visible to users
class Buffer : public ADatatype {
    virtual std::shared_ptr<dai::RawBuffer> serialize() const;

   public:
    Buffer();
    Buffer(std::shared_ptr<dai::RawBuffer> ptr);
    ~Buffer() = default;

    // helpers
    std::vector<std::uint8_t>& getData();
    void setData(std::vector<std::uint8_t> data);
};

}  // namespace dai
