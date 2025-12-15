#pragma once
#include <memory>
#include <vector>

#include "depthai/pipeline/datatype/DatatypeEnum.hpp"
#include "depthai/utility/Memory.hpp"
#include "depthai/utility/Serialization.hpp"
#include "depthai/utility/VectorMemory.hpp"

namespace dai {

/// Abstract message
class ADatatype {
   protected:
    friend class DataInputQueue;
    friend class StreamMessageParser;

   public:
#ifdef DEPTHAI_MESSAGES_NO_HEAP
    explicit ADatatype() = default;
#else
    explicit ADatatype() : data{std::make_shared<VectorMemory>(std::vector<uint8_t>())} {};
#endif

    virtual ~ADatatype();
    virtual void serialize(std::vector<std::uint8_t>& metadata, DatatypeEnum& datatype) const;

    /**
     * @brief Get the datatype of this specific message
     * @return DatatypeEnum
     */
    virtual DatatypeEnum getDatatype() const {
        return DatatypeEnum::ADatatype;
    }

    std::shared_ptr<Memory> data;
};

}  // namespace dai