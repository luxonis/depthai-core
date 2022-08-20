#include "depthai/pipeline/datatype/SystemInformationS3.hpp"

namespace dai {

std::shared_ptr<RawBuffer> SystemInformationS3::serialize() const {
    return raw;
}

SystemInformationS3::SystemInformationS3()
    : Buffer(std::make_shared<RawSystemInformationS3>()),
      systemInformation(*dynamic_cast<RawSystemInformationS3*>(raw.get())),
      ddrMemoryUsage(systemInformation.ddrMemoryUsage),
      cpuAvgUsage(systemInformation.cpuAvgUsage),
      cpuUsages(systemInformation.cpuUsages),
      chipTemperature(systemInformation.chipTemperature) {}

SystemInformationS3::SystemInformationS3(std::shared_ptr<RawSystemInformationS3> ptr)
    : Buffer(std::move(ptr)),
      systemInformation(*dynamic_cast<RawSystemInformationS3*>(raw.get())),
      ddrMemoryUsage(systemInformation.ddrMemoryUsage),
      cpuAvgUsage(systemInformation.cpuAvgUsage),
      cpuUsages(systemInformation.cpuUsages),
      chipTemperature(systemInformation.chipTemperature) {}

}  // namespace dai
