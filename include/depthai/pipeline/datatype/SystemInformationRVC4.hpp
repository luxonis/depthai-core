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
 * SystemInformation message.
 * Carries memory and CPU usage, chip temperatures, accelerator activity and power readings.
 */
class SystemInformationRVC4 : public Buffer {
   public:
    /**
     * Construct SystemInformation message.
     */
    SystemInformationRVC4() = default;
    virtual ~SystemInformationRVC4();

    MemoryInfo ddrMemoryUsage;
    int64_t processMemoryUsage = 0;
    CpuUsage cpuAvgUsage;
    CpuUsage processCpuAvgUsage;
    std::vector<CpuUsage> cpuUsages;
    ChipTemperatureRVC4 chipTemperature;

    double gpuUtilizationPercent = -1;
    double dspHvxActivityPercent = -1;
    double dspHmxActivityPercent = -1;
    double dspQ6LoadKcps = -1;
    double powerSystemWatts = -1;
    double powerProcessorWatts = -1;
    double powerPoeWatts = -1;
    double powerTofWatts = -1;

    void serialize(std::vector<std::uint8_t>& metadata, DatatypeEnum& datatype) const override;
    DatatypeEnum getDatatype() const override {
        return DatatypeEnum::SystemInformationRVC4;
    }

    DEPTHAI_SERIALIZE_DECL(SystemInformationRVC4,
                           ddrMemoryUsage,
                           processMemoryUsage,
                           cpuAvgUsage,
                           processCpuAvgUsage,
                           cpuUsages,
                           chipTemperature,
                           gpuUtilizationPercent,
                           dspHvxActivityPercent,
                           dspHmxActivityPercent,
                           dspQ6LoadKcps,
                           powerSystemWatts,
                           powerProcessorWatts,
                           powerPoeWatts,
                           powerTofWatts);
};

}  // namespace dai
