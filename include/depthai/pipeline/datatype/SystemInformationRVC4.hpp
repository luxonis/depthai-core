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
 * SystemInformation message for RVC4 devices.
 * Carries memory usage, CPU usage, chip temperatures, accelerator activity and power rails. Accelerator readings come from vendor counters;
 * their sampling-window duration is not exposed and must not be assumed to match the SystemLogger output interval. GPU utilization covers the
 * whole GPU. DSP activity is not awake residency or total NPU utilization. Power values are separate rail readings and are not additive.
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

    /** Whole-GPU utilization percentage from the vendor sampling window; not per-engine utilization; -1 when unavailable. */
    double gpuUtilizationPercent = -1;
    /** DSP HVX activity percentage from the vendor sampling window; activity is not awake residency or total NPU utilization; -1 when unavailable. */
    double dspHvxActivityPercent = -1;
    /** DSP HMX activity percentage from the vendor sampling window; activity is not awake residency or total NPU utilization; -1 when unavailable. */
    double dspHmxActivityPercent = -1;
    /** DSP Q6 load in KCPS from the vendor sampling window; not awake residency or total NPU utilization; -1 when unavailable. */
    double dspQ6LoadKcps = -1;
    /** vdd_5v0 system rail in watts, converted from microwatts; independent reading, not additive with other rails; -1 when unavailable. */
    double powerSystemWatts = -1;
    /** vdd_3v9 processor rail in watts, converted from microwatts; independent reading, not additive with other rails; -1 when unavailable. */
    double powerProcessorWatts = -1;
    /** PoE power rail in watts; independent rail reading, not additive with other rails; -1 when unavailable. */
    double powerPoeWatts = -1;
    /** ToF power rail in watts; independent rail reading, not additive with other rails; -1 when unavailable. */
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
