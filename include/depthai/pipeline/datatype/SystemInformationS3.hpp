#pragma once

#include <chrono>
#include <unordered_map>
#include <vector>

#include "depthai-shared/datatype/RawSystemInformationS3.hpp"
#include "depthai/pipeline/datatype/Buffer.hpp"
namespace dai {

/**
 * SystemInformation message for series 3 devices.
 * Carries memory usage, cpu usage and chip temperatures.
 */
class SystemInformationS3 : public Buffer {
    Serialized serialize() const override;
    RawSystemInformationS3& systemInformation;

   public:
    /**
     * Construct SystemInformation message.
     */
    SystemInformationS3();
    explicit SystemInformationS3(std::shared_ptr<RawSystemInformationS3> ptr);
    virtual ~SystemInformationS3() = default;

    MemoryInfo& ddrMemoryUsage;
    CpuUsage& cpuAvgUsage;
    std::vector<CpuUsage>& cpuUsages;
    ChipTemperatureS3& chipTemperature;
};

}  // namespace dai