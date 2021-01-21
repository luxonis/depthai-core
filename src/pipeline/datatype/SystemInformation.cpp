#include "depthai/pipeline/datatype/SystemInformation.hpp"

namespace dai {

std::shared_ptr<RawBuffer> SystemInformation::serialize() const {
    return raw;
}

SystemInformation::SystemInformation()
    : Buffer(std::make_shared<RawSystemInformation>()),
      systemInformation(*dynamic_cast<RawSystemInformation*>(raw.get())),
      memoryDdrUsage(systemInformation.memoryDdrUsage),
      memoryLeonOsUsage(systemInformation.memoryLeonOsUsage),
      memoryLeonRtUsage(systemInformation.memoryLeonRtUsage),
      cpuLeonOsUsage(systemInformation.cpuLeonOsUsage),
      cpuLeonRtUsage(systemInformation.cpuLeonRtUsage),
      chipTemperature(systemInformation.chipTemperature) {}

SystemInformation::SystemInformation(std::shared_ptr<RawSystemInformation> ptr)
    : Buffer(std::move(ptr)),
      systemInformation(*dynamic_cast<RawSystemInformation*>(raw.get())),
      memoryDdrUsage(systemInformation.memoryDdrUsage),
      memoryLeonOsUsage(systemInformation.memoryLeonOsUsage),
      memoryLeonRtUsage(systemInformation.memoryLeonRtUsage),
      cpuLeonOsUsage(systemInformation.cpuLeonOsUsage),
      cpuLeonRtUsage(systemInformation.cpuLeonRtUsage),
      chipTemperature(systemInformation.chipTemperature) {}

}  // namespace dai
