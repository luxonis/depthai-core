#pragma once
#include <cstdint>
#include <vector>

#include "depthai-shared/common/ChipTemperatureS3.hpp"
#include "depthai-shared/common/CpuUsage.hpp"
#include "depthai-shared/common/MemoryInfo.hpp"
#include "depthai-shared/datatype/DatatypeEnum.hpp"
#include "depthai-shared/datatype/RawBuffer.hpp"
#include "depthai-shared/utility/Serialization.hpp"

namespace dai {
/**
 * System information of device for series 3 devices
 *
 * Memory usage, cpu usage and chip temperature
 */
struct RawSystemInformationS3 : public RawBuffer {
    // DDR memory usage
    MemoryInfo ddrMemoryUsage;
    // Cpu usage
    CpuUsage cpuAvgUsage;
    // Each core
    std::vector<CpuUsage> cpuUsages;
    // Chip temperatures
    ChipTemperatureS3 chipTemperature;

    void serialize(std::vector<std::uint8_t>& metadata, DatatypeEnum& datatype) const override {
        metadata = utility::serialize(*this);
        datatype = DatatypeEnum::SystemInformationS3;
    };

    DEPTHAI_SERIALIZE(RawSystemInformationS3, ddrMemoryUsage, cpuAvgUsage, cpuUsages, chipTemperature);
};

}  // namespace dai