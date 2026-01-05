#pragma once

#include <vector>

#include "depthai/common/ChipTemperatureRVC4.hpp"
#include "depthai/common/CpuUsage.hpp"
#include "depthai/common/MemoryInfo.hpp"
#include "depthai/pipeline/datatype/Buffer.hpp"
#include "depthai/pipeline/datatype/DatatypeEnum.hpp"
#include "depthai/utility/Serialization.hpp"
namespace dai {

/**
 * SystemInformation message
 * Carries memory usage, cpu usage and chip temperatures.
 */
class SystemInformationRVC4 : public Buffer {
   public:
    /**
     * Construct SystemInformation message.
     */
    SystemInformationRVC4() = default;
    virtual ~SystemInformationRVC4();

    MemoryInfo ddrMemoryUsage;
    CpuUsage cpuAvgUsage;
    std::vector<CpuUsage> cpuUsages;
    ChipTemperatureRVC4 chipTemperature;

    void serialize(std::vector<std::uint8_t>& metadata, DatatypeEnum& datatype) const override;
    DatatypeEnum getDatatype() const override {
        return DatatypeEnum::SystemInformationRVC4;
    }

    DEPTHAI_SERIALIZE(SystemInformationRVC4, ddrMemoryUsage, cpuAvgUsage, cpuUsages, chipTemperature);
};

}  // namespace dai