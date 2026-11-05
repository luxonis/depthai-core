#include "depthai/pipeline/datatype/SystemInformationRVC4.hpp"

#include <catch2/catch_all.hpp>
#include <stdexcept>

#include "depthai/utility/Serialization.hpp"

namespace {

// Wire schema shipped before accelerator and power fields were appended.
struct LegacySystemInformationRVC4 {
    dai::MemoryInfo ddrMemoryUsage;
    int64_t processMemoryUsage = 0;
    dai::CpuUsage cpuAvgUsage;
    dai::CpuUsage processCpuAvgUsage;
    std::vector<dai::CpuUsage> cpuUsages;
    dai::ChipTemperatureRVC4 chipTemperature;

    DEPTHAI_SERIALIZE(LegacySystemInformationRVC4, ddrMemoryUsage, processMemoryUsage, cpuAvgUsage, processCpuAvgUsage, cpuUsages, chipTemperature);
};

}  // namespace

TEST_CASE("SystemInformationRVC4 appended fields serialize and roundtrip") {
    dai::SystemInformationRVC4 message;
    REQUIRE(message.gpuUtilizationPercent == -1);
    REQUIRE(message.dspHvxActivityPercent == -1);
    REQUIRE(message.dspHmxActivityPercent == -1);
    REQUIRE(message.dspQ6LoadKcps == -1);
    REQUIRE(message.powerSystemWatts == -1);
    REQUIRE(message.powerProcessorWatts == -1);
    REQUIRE(message.powerPoeWatts == -1);
    REQUIRE(message.powerTofWatts == -1);
    message.gpuUtilizationPercent = 23.5;
    message.dspHvxActivityPercent = 10.25;
    message.dspHmxActivityPercent = 4.75;
    message.dspQ6LoadKcps = 321.0;
    message.powerSystemWatts = 2.1;
    message.powerProcessorWatts = 1.2;
    message.powerPoeWatts = 0.7;
    message.powerTofWatts = 0.3;

    const auto bytes = dai::utility::serialize(message);
    dai::SystemInformationRVC4 restored;
    REQUIRE(dai::utility::deserialize(bytes, restored));
    REQUIRE(restored.gpuUtilizationPercent == Catch::Approx(23.5));
    REQUIRE(restored.dspHvxActivityPercent == Catch::Approx(10.25));
    REQUIRE(restored.dspHmxActivityPercent == Catch::Approx(4.75));
    REQUIRE(restored.dspQ6LoadKcps == Catch::Approx(321.0));
    REQUIRE(restored.powerSystemWatts == Catch::Approx(2.1));
    REQUIRE(restored.powerProcessorWatts == Catch::Approx(1.2));
    REQUIRE(restored.powerPoeWatts == Catch::Approx(0.7));
    REQUIRE(restored.powerTofWatts == Catch::Approx(0.3));
}

TEST_CASE("SystemInformationRVC4 old wire payload is not compatible with the extended libnop schema") {
    LegacySystemInformationRVC4 oldMessage;
    oldMessage.processMemoryUsage = 12345;
    oldMessage.cpuAvgUsage.average = 0.5f;

    const auto oldBytes = dai::utility::serialize(oldMessage);
    dai::SystemInformationRVC4 decoded;
    bool decodedSuccessfully = false;
    try {
        decodedSuccessfully = dai::utility::deserialize(oldBytes, decoded);
    } catch(const std::runtime_error&) {
        // In exception-enabled builds libnop reports a truncated struct by throwing.
    }
    REQUIRE_FALSE(decodedSuccessfully);
}

TEST_CASE("SystemInformationRVC4 extended wire payload is not compatible with the old libnop schema") {
    dai::SystemInformationRVC4 newMessage;
    newMessage.processMemoryUsage = 12345;
    newMessage.cpuAvgUsage.average = 0.5f;
    newMessage.gpuUtilizationPercent = 50;

    const auto newBytes = dai::utility::serialize(newMessage);
    LegacySystemInformationRVC4 oldDecoder;
    bool decodedSuccessfully = false;
    try {
        decodedSuccessfully = dai::utility::deserialize(newBytes, oldDecoder);
    } catch(const std::runtime_error&) {
        // In exception-enabled builds libnop reports a struct member-count mismatch by throwing.
    }
    REQUIRE_FALSE(decodedSuccessfully);
}
