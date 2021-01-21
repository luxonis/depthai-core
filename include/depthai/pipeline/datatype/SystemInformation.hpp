#pragma once

#include <chrono>
#include <unordered_map>
#include <vector>

#include "depthai-shared/datatype/RawSystemInformation.hpp"
#include "depthai/pipeline/datatype/Buffer.hpp"
namespace dai {

// protected inheritance, so serialize isn't visible to users
class SystemInformation : public Buffer {
    std::shared_ptr<RawBuffer> serialize() const override;
    RawSystemInformation& systemInformation;

   public:
    SystemInformation();
    explicit SystemInformation(std::shared_ptr<RawSystemInformation> ptr);
    virtual ~SystemInformation() = default;

    MemoryInfo& memoryDdrUsage;
    MemoryInfo& memoryLeonOsUsage;
    MemoryInfo& memoryLeonRtUsage;
    CpuUsage& cpuLeonOsUsage;
    CpuUsage& cpuLeonRtUsage;
    ChipTemperature& chipTemperature;
};

}  // namespace dai
