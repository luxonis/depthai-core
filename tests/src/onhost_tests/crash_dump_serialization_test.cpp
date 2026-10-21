#include <catch2/catch_all.hpp>
#include <cstdint>
#include <memory>
#include <nlohmann/json.hpp>
#include <string>
#include <vector>

#include "depthai/device/CrashDump.hpp"

namespace {

// UTF-8 encoding of U+FFFD REPLACEMENT CHARACTER
const std::string REPLACEMENT_CHARACTER = "\xEF\xBF\xBD";
// UTF-8 encoding of U+00B0 DEGREE SIGN (valid multi byte UTF-8)
const std::string DEGREE_SIGN = "\xC2\xB0";

std::string withInvalidByte(const std::string& prefix, uint8_t invalidByte, const std::string& suffix) {
    return prefix + std::string(1, static_cast<char>(invalidByte)) + suffix;
}

}  // namespace

// The crash report strings (device prints, thread names, ...) come from the device as raw bytes.
// They are not always valid UTF-8, which must not prevent the crash dump archive from being written.
TEST_CASE("RVC2 crash dump with non UTF-8 device strings can be archived and loaded") {
    dai::CrashDumpRVC2 dump;
    dump.deviceId = "14442C104149CCD600";
    dump.crashdumpTimestamp = "2026-10-17 04:57:06";
    dump.depthaiCommitHash = "0123456789abcdef";

    dai::CrashDumpRVC2::CrashReport report;
    report.processor = dai::ProcessorType::LEON_CSS;
    report.errorSource = "RTEMS_FATAL";
    dai::CrashDumpRVC2::CrashReport::ThreadCallstack thread;
    thread.threadId = 1;
    thread.threadName = withInvalidByte("Thread", 0xC0, "");
    report.threadCallstack.push_back(thread);
    report.prints.push_back(withInvalidByte("[LeonCss] device print number 1 ", 0xC9, " end"));
    // Valid multi byte UTF-8 (degree sign) must stay as it is
    report.prints.push_back("[LeonCss] temperature 45 " + DEGREE_SIGN + "C");
    dump.crashReports.crashReports.push_back(report);
    dump.crashReports.deviceId = dump.deviceId;

    std::vector<uint8_t> bytes;
    REQUIRE_NOTHROW(bytes = dump.toBytes());
    REQUIRE_FALSE(bytes.empty());

    auto loaded = dai::CrashDump::fromBytes(bytes);
    auto* loadedRvc2 = dynamic_cast<dai::CrashDumpRVC2*>(loaded.get());
    REQUIRE(loadedRvc2 != nullptr);
    REQUIRE(loadedRvc2->deviceId == dump.deviceId);
    REQUIRE(loadedRvc2->crashReports.crashReports.size() == 1);

    // The invalid byte is replaced, the valid text around it is kept
    const auto& loadedReport = loadedRvc2->crashReports.crashReports.front();
    REQUIRE(loadedReport.errorSource == "RTEMS_FATAL");
    REQUIRE(loadedReport.prints.size() == 2);
    REQUIRE(loadedReport.prints.at(0) == "[LeonCss] device print number 1 " + REPLACEMENT_CHARACTER + " end");
    REQUIRE(loadedReport.prints.at(1) == report.prints.at(1));
    REQUIRE(loadedReport.threadCallstack.size() == 1);
    REQUIRE(loadedReport.threadCallstack.front().threadName == "Thread" + REPLACEMENT_CHARACTER);
}

// The extra data is set by the user (crashdump_test.cpp uses this to inject an archive failure).
// It is still serialized strictly.
TEST_CASE("Crash dump extra data with non UTF-8 strings fails archive creation") {
    dai::CrashDumpRVC2 dump;
    dump["invalidUtf8"] = std::string(1, static_cast<char>(0xFF));
    REQUIRE_THROWS_AS(dump.toBytes(), nlohmann::json::type_error);
}
