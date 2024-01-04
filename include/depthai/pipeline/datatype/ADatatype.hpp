#pragma once
#include <memory>
#include <vector>

#include "depthai/pipeline/datatype/DatatypeEnum.hpp"
#include "depthai/utility/Memory.hpp"
#include "depthai/utility/VectorMemory.hpp"

namespace dai {

/// Abstract message
class ADatatype {
   protected:
    friend class DataInputQueue;
    friend class StreamMessageParser;

   public:
    explicit ADatatype() : data{std::make_shared<VectorMemory>(std::vector<uint8_t>())} {}
    virtual ~ADatatype() = default;

    virtual void serialize(std::vector<std::uint8_t>& metadata, DatatypeEnum& datatype) const {
        (void)metadata;
        datatype = DatatypeEnum::ADatatype;
    };

    std::shared_ptr<Memory> data;
};

}  // namespace dai
