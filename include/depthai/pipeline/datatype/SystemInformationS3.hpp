#pragma once

#include <vector>

#include "depthai/common/ChipTemperatureS3.hpp"
#include "depthai/common/CpuUsage.hpp"
#include "depthai/common/MemoryInfo.hpp"
#include "depthai/pipeline/datatype/DatatypeEnum.hpp"
#include "depthai/pipeline/datatype/Buffer.hpp"
#include "depthai/utility/Serialization.hpp"
namespace dai {

/**
 * SystemInformation message for series 3 devices.
 * Carries memory usage, cpu usage and chip temperatures.
 */
class SystemInformationS3 : public Buffer {
   public:
    /**
     * Construct SystemInformation message.
     */
    SystemInformationS3() = default;
    virtual ~SystemInformationS3() = default;

    MemoryInfo ddrMemoryUsage;
    CpuUsage cpuAvgUsage;
    std::vector<CpuUsage> cpuUsages;
    ChipTemperatureS3 chipTemperature;


    void serialize(std::vector<std::uint8_t>& metadata, DatatypeEnum& datatype) const override {
        metadata = utility::serialize(*this);
        datatype = DatatypeEnum::SystemInformationS3;
    };

    DEPTHAI_SERIALIZE(SystemInformationS3, ddrMemoryUsage, cpuAvgUsage, cpuUsages, chipTemperature);
};

}  // namespace dai