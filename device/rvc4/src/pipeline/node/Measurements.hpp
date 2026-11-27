#pragma once
#include <spdlog/logger.h>

#include <chrono>
#include <string>
#include <vector>

#include "depthai/pipeline/datatype/SystemInformationRVC4.hpp"

class Measurements {
   public:
    Measurements() = delete;  // Avoid checking if logger is present every time
    Measurements(std::shared_ptr<spdlog::logger> pLogger);
    static constexpr int NUM_CPU_STATES = 10;
    enum cpuStates { S_USER = 0, S_NICE, S_SYSTEM, S_IDLE, S_IOWAIT, S_IRQ, S_SOFTIRQ, S_STEAL, S_GUEST, S_GUEST_NICE };
    struct cpuData {
        std::string cpu;
        size_t times[NUM_CPU_STATES];
    };

    void readStatsCpu(std::vector<cpuData>& entries, cpuData& data);
    float getAvgCpuLoad(const cpuData& entry1, const cpuData& entry2);
    float getProcessAvgCpuLoad(const cpuData& totalStart, const cpuData& totalEnd);
    void printCpuStats(const std::vector<cpuData>& entries1, const std::vector<cpuData>& entries2);
    dai::MemoryInfo getMemoryInfo();
    int64_t getProcessMemoryInfo();
    dai::ChipTemperatureRVC4 getChipTemperature();
    void readSystemInformation(dai::SystemInformationRVC4& pSysInfo,
                               const std::chrono::duration<float> sleepPeriod,
                               const std::vector<Measurements::cpuData>& entriesStart,
                               const std::vector<Measurements::cpuData>& entriesEnd,
                               const cpuData& totalStart,
                               const cpuData& totalEnd);

    std::shared_ptr<spdlog::logger> logger;

   private:
    size_t getCpuIdleTime(const cpuData& e);
    size_t getCpuActiveTime(const cpuData& e);
    size_t readProcessCpuTicks();
    size_t prevProcessCpuTicks = 0;
};
