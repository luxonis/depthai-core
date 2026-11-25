#include "Measurements.hpp"

#include <unistd.h>

#include <chrono>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <map>
#include <regex>
#include <sstream>
#include <string>

#include "SystemResourceCounters.hpp"
#include "utilities/OSUtils.hpp"

Measurements::Measurements(std::shared_ptr<spdlog::logger> pLogger) {
    logger = pLogger;
    prevProcessCpuTicks = readProcessCpuTicks();
}

size_t Measurements::getCpuIdleTime(const cpuData& e) {
    return e.times[S_IDLE] + e.times[S_IOWAIT];
}

size_t Measurements::getCpuActiveTime(const cpuData& e) {
    return e.times[S_USER] + e.times[S_NICE] + e.times[S_SYSTEM] + e.times[S_IRQ] + e.times[S_SOFTIRQ] + e.times[S_STEAL] + e.times[S_GUEST]
           + e.times[S_GUEST_NICE];
}

float Measurements::getAvgCpuLoad(const cpuData& entry1, const cpuData& entry2) {
    const float activeTime = static_cast<float>(getCpuActiveTime(entry2) - getCpuActiveTime(entry1));
    const float idleTime = static_cast<float>(getCpuIdleTime(entry2) - getCpuIdleTime(entry1));
    const float totalTime = activeTime + idleTime;

    return activeTime / totalTime;
}

float Measurements::getProcessAvgCpuLoad(const cpuData& totalStart, const cpuData& totalEnd) {
    const size_t currentProcessCpuTicks = readProcessCpuTicks();
    const float activeTime = static_cast<float>(getCpuActiveTime(totalEnd) - getCpuActiveTime(totalStart));
    const float idleTime = static_cast<float>(getCpuIdleTime(totalEnd) - getCpuIdleTime(totalStart));
    const float systemDelta = activeTime + idleTime;

    float average = 0.0f;
    if(systemDelta > 0.0f && prevProcessCpuTicks > 0) {
        average = static_cast<float>(currentProcessCpuTicks - prevProcessCpuTicks) / systemDelta;
    }
    prevProcessCpuTicks = currentProcessCpuTicks;
    return average;
}

size_t Measurements::readProcessCpuTicks() {
    pid_t pid = getpid();
    std::string statPath = "/proc/" + std::to_string(pid) + "/stat";
    std::ifstream statFile(statPath);

    if(!statFile) {
        logger->error("Failed to open {}", statPath);
        return 0;
    }

    std::string line;
    if(!std::getline(statFile, line)) {
        logger->error("Failed to read from {}", statPath);
        return 0;
    }

    // /proc/<pid>/stat format: pid (comm) state ppid pgrp session tty_nr tpgid flags ...
    // Fields 14 and 15 are utime and stime (1-indexed)
    // Need to skip past the command name (which may contain spaces) in parentheses
    size_t startComm = line.find('(');
    size_t endComm = line.rfind(')');
    if(startComm == std::string::npos || endComm == std::string::npos) {
        logger->error("Invalid format in {}", statPath);
        return 0;
    }

    // Parse fields after (comm)
    std::istringstream iss(line.substr(endComm + 2));  // Skip ") "
    std::string state;
    size_t field;
    iss >> state;  // Field 3: state

    // Skip fields 4-13 (10 fields)
    for(int i = 4; i <= 13; ++i) {
        iss >> field;
    }

    // Read utime (field 14) and stime (field 15)
    size_t utime, stime;
    iss >> utime >> stime;

    return utime + stime;
}

void Measurements::readStatsCpu(std::vector<cpuData>& entries, cpuData& total) {
    entries.clear();
    std::string fileStatName = "/proc/stat";
    std::ifstream fileStat(fileStatName);

    if(!fileStat) {  // Check if file was opened successfully
        logger->warn("Cannot get info about CPU load");
        logger->warn("Couldn't open file: {}", fileStatName);
        return;
    }

    std::string line;
    const std::string strCpu("cpu");
    const std::size_t lenStrCpu = strCpu.size();
    const std::string strTot("tot");

    while(std::getline(fileStat, line)) {
        // cpu stats line found
        if(!line.compare(0, lenStrCpu, strCpu)) {
            std::istringstream ss(line);

            // store entry
            entries.emplace_back(cpuData());
            cpuData& entry = entries.back();

            // read cpu label
            ss >> entry.cpu;

            if(entry.cpu.size() > lenStrCpu)
                entry.cpu.erase(0, lenStrCpu);
            else
                entry.cpu = strTot;

            // read times
            for(int i = 0; i < NUM_CPU_STATES; ++i) {
                ss >> entry.times[i];
            }

            // save total separately
            if(entry.cpu == strTot) {
                total = entry;
                entries.pop_back();  // Store total separately
            }
        }
    }
}

void Measurements::printCpuStats(const std::vector<cpuData>& entries1, const std::vector<cpuData>& entries2) {
    const size_t numEntries = entries1.size();

    for(size_t i = 0; i < numEntries; ++i) {
        const cpuData& e1 = entries1[i];
        const cpuData& e2 = entries2[i];

        std::cout.width(3);
        std::cout << e1.cpu << "] ";

        const float activeTime = static_cast<float>(getCpuActiveTime(e2) - getCpuActiveTime(e1));
        const float idleTime = static_cast<float>(getCpuIdleTime(e2) - getCpuIdleTime(e1));
        const float totalTime = activeTime + idleTime;

        std::cout << "active: ";
        std::cout.setf(std::ios::fixed, std::ios::floatfield);
        std::cout.width(6);
        std::cout.precision(2);
        std::cout << (100.f * activeTime / totalTime) << "%";

        std::cout << " - idle: ";
        std::cout.setf(std::ios::fixed, std::ios::floatfield);
        std::cout.width(6);
        std::cout.precision(2);
        std::cout << (100.f * idleTime / totalTime) << "%" << std::endl;
    }
}

dai::MemoryInfo Measurements::getMemoryInfo() {
    dai::MemoryInfo memoryInfo;
    const std::string meminfoFileName("/proc/meminfo");
    std::ifstream memInfoFile(meminfoFileName);

    if(!memInfoFile) {
        logger->warn("Can't get data about memory usage");
        logger->warn("Couldn't open file: {}", meminfoFileName);
        return memoryInfo;
    }
    std::string line;
    std::map<std::string, int64_t> params;
    while(std::getline(memInfoFile, line)) {
        std::istringstream iss(line);
        std::string name;
        std::string value;
        if(iss >> name >> value) {
            name.pop_back();
            params[name] = std::stol(value) * 1024;  // Convert to bytes from kilobytes
        } else {
            std::cout << meminfoFileName << "has wrong format: " << line << '\n';
            return memoryInfo;
        }
    }
    try {
        memoryInfo.remaining = params.at("MemAvailable");
        memoryInfo.total = params.at("MemTotal");
        memoryInfo.used = memoryInfo.total - memoryInfo.remaining;
    } catch(std::out_of_range& ex) {
        logger->warn("Couldn't read memory info: {}", ex.what());
        return dai::MemoryInfo{};
    }
    return memoryInfo;
}

int64_t Measurements::getProcessMemoryInfo() {
    const auto memory = dai::gate::getProcessMemoryUsage();
    if(!memory) logger->error("Failed to read process memory usage");
    return memory.value_or(0);
}

dai::ChipTemperatureRVC4 Measurements::getChipTemperature() {
    dai::ChipTemperatureRVC4 chipTemp;

    std::map<std::string, float> chipTemps = {{"cpuss-0", 0.0f}, {"gpuss-0", 0.0f}, {"mdmss-0", 0.0f}, {"video", 0.0f}, {"ddr", 0.0f}, {"camera-0", 0.0f}};

    // Regular expression to match thermal_zone directories
    std::regex thermalZoneRegex("thermal_zone\\d+");
    for(const auto& entry : std::filesystem::directory_iterator("/sys/class/thermal")) {
        if(!entry.is_directory()) {
            continue;
        }

        std::string dirname = entry.path().filename().string();

        // Check if directory matches thermal_zone pattern
        if(!std::regex_match(dirname, thermalZoneRegex)) {
            continue;
        }

        std::string fileTypeName = entry.path().string() + "/type";
        std::string fileTempName = entry.path().string() + "/temp";

        std::string type;
        std::string value;
        std::ifstream fileType(fileTypeName);
        std::ifstream fileTemp(fileTempName);

        if(!fileType || !fileTemp || !(fileType >> type) || !(fileTemp >> value) || value.empty()) {
            continue;
        }

        if(chipTemps.find(type) != chipTemps.end()) {
            chipTemps[type] = std::stoi(value) / 1000.0f;
        }
    }

    try {
        chipTemp.cpuss = chipTemps.at("cpuss-0");
        chipTemp.gpuss = chipTemps.at("gpuss-0");
        chipTemp.mdmss = chipTemps.at("mdmss-0");
        chipTemp.video = chipTemps.at("video");
        chipTemp.ddr = chipTemps.at("ddr");
        chipTemp.camera = chipTemps.at("camera-0");
        chipTemp.average = .0f;  // Init average to 0
        for(auto& el : chipTemps) {
            chipTemp.average += el.second;
        }
        chipTemp.average = chipTemp.average / chipTemps.size();
    } catch(std::out_of_range& ex) {
        logger->warn("Couldn't read temperature: {}", ex.what());
        return dai::ChipTemperatureRVC4{};
    }
    return chipTemp;
}

void Measurements::readSystemInformation(dai::SystemInformationRVC4& pSysInfo,
                                         const std::chrono::duration<float> sleepPeriod,
                                         const std::vector<Measurements::cpuData>& entriesStart,
                                         const std::vector<Measurements::cpuData>& entriesEnd,
                                         const cpuData& totalStart,
                                         const cpuData& totalEnd) {
    using namespace std::chrono;

    // Parse CPU total load
    dai::CpuUsage cpuAvgUsage;
    cpuAvgUsage.average = getAvgCpuLoad(totalStart, totalEnd);  // Between 0..1
    cpuAvgUsage.msTime = static_cast<int32_t>(duration_cast<milliseconds>(sleepPeriod).count());
    pSysInfo.cpuAvgUsage = cpuAvgUsage;

    // Parse process CPU load
    dai::CpuUsage processCpuAvgUsage;
    processCpuAvgUsage.average = getProcessAvgCpuLoad(totalStart, totalEnd);  // Between 0..1
    processCpuAvgUsage.msTime = static_cast<int32_t>(duration_cast<milliseconds>(sleepPeriod).count());
    pSysInfo.processCpuAvgUsage = processCpuAvgUsage;

    // Parse CPU load per thread
    for(int i = 0; i < entriesStart.size(); i++) {
        dai::CpuUsage cpuUsage;
        auto start = entriesStart[i];
        auto end = entriesEnd[i];
        cpuUsage.average = getAvgCpuLoad(start, end);  // Between 0..1
        cpuUsage.msTime = static_cast<int32_t>(duration_cast<milliseconds>(sleepPeriod).count());
        pSysInfo.cpuUsages.push_back(cpuUsage);
    }

    // Get chip temperature and memory usage
    pSysInfo.ddrMemoryUsage = getMemoryInfo();
    pSysInfo.processMemoryUsage = getProcessMemoryInfo();
    pSysInfo.chipTemperature = getChipTemperature();

    const auto resourceCounters = system_resource_counters::Collector{}.read();
    pSysInfo.gpuUtilizationPercent = resourceCounters.gpuUtilizationPercent;
    pSysInfo.dspHvxActivityPercent = resourceCounters.dspHvxActivityPercent;
    pSysInfo.dspHmxActivityPercent = resourceCounters.dspHmxActivityPercent;
    pSysInfo.dspQ6LoadKcps = resourceCounters.dspQ6LoadKcps;
    pSysInfo.powerSystemWatts = resourceCounters.powerSystemWatts;
    pSysInfo.powerProcessorWatts = resourceCounters.powerProcessorWatts;
    pSysInfo.powerPoeWatts = resourceCounters.powerPoeWatts;
    pSysInfo.powerTofWatts = resourceCounters.powerTofWatts;
}
