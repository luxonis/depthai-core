#include "depthai/pipeline/datatype/SystemInformationS3.hpp"

namespace dai {

SystemInformationS3::Serialized SystemInformationS3::serialize() const {
    return {data, raw};
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
