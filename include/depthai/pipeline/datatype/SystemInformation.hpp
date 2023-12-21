#pragma once

#include <chrono>
#include <unordered_map>
#include <vector>

#include "depthai/common/ChipTemperature.hpp"
#include "depthai/common/CpuUsage.hpp"
#include "depthai/common/MemoryInfo.hpp"
#include "depthai/pipeline/datatype/DatatypeEnum.hpp"
#include "depthai/utility/Serialization.hpp"
#include "depthai/pipeline/datatype/Buffer.hpp"
namespace dai {

/**
 * SystemInformation message. Carries memory usage, cpu usage and chip temperatures.
 */
class SystemInformation : public Buffer {
   public:
    /**
     * Construct SystemInformation message.
     */
    SystemInformation() = default;
    virtual ~SystemInformation() = default;

    MemoryInfo ddrMemoryUsage;
    MemoryInfo cmxMemoryUsage;
    MemoryInfo leonCssMemoryUsage;
    MemoryInfo leonMssMemoryUsage;
    CpuUsage leonCssCpuUsage;
    CpuUsage leonMssCpuUsage;
    ChipTemperature chipTemperature;

    void serialize(std::vector<std::uint8_t>& metadata, DatatypeEnum& datatype) const override {
        metadata = utility::serialize(*this);
        datatype = DatatypeEnum::SystemInformation;
    };

    DEPTHAI_SERIALIZE(SystemInformation, ddrMemoryUsage, cmxMemoryUsage, leonCssMemoryUsage, leonMssMemoryUsage, leonCssCpuUsage, leonMssCpuUsage, chipTemperature);
};

}  // namespace dai
