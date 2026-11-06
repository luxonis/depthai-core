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

    // Accelerator and power readings use -1 when unavailable.
    // Vendor activity windows may differ from the logger interval; DSP activity is not awake residency or total NPU utilization.
    // Power rails are separate readings and must not be summed.

    /** Whole-GPU utilization (%). */
    double gpuUtilizationPercent = -1;
    /** DSP HVX activity (%). */
    double dspHvxActivityPercent = -1;
    /** DSP HMX activity (%). */
    double dspHmxActivityPercent = -1;
    /** DSP Q6 load (KCPS). */
    double dspQ6LoadKcps = -1;
    /** System rail power (vdd_5v0), in watts. */
    double powerSystemWatts = -1;
    /** Processor rail power (vdd_3v9), in watts. */
    double powerProcessorWatts = -1;
    /** PoE rail power, in watts. */
    double powerPoeWatts = -1;
    /** ToF rail power, in watts. */
    double powerTofWatts = -1;

    void serialize(std::vector<std::uint8_t>& metadata, DatatypeEnum& datatype) const override;
    DatatypeEnum getDatatype() const override {
        return DatatypeEnum::SystemInformationRVC4;
    }

    DEPTHAI_SERIALIZE(SystemInformationRVC4,
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
