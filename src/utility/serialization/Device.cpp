#include "depthai/device/BoardConfig.hpp"
#include "depthai/device/CalibrationHandler.hpp"
#include "depthai/device/CrashDump.hpp"
#include "utility/serialization/SerializationDefine.hpp"

namespace dai {

DEPTHAI_SERIALIZE_EXT_DEFINE(BoardConfig::USB)
DEPTHAI_SERIALIZE_EXT_DEFINE(BoardConfig::Network)
DEPTHAI_SERIALIZE_EXT_DEFINE(BoardConfig::GPIO)
DEPTHAI_SERIALIZE_EXT_DEFINE(BoardConfig::UART)
DEPTHAI_SERIALIZE_EXT_DEFINE(BoardConfig::Camera)
DEPTHAI_SERIALIZE_EXT_DEFINE(BoardConfig::IMU)
DEPTHAI_SERIALIZE_EXT_DEFINE(BoardConfig::UVC)
DEPTHAI_SERIALIZE_EXT_DEFINE(BoardConfig)
DEPTHAI_SERIALIZE_DEFINE(CalibrationHandler)
DEPTHAI_SERIALIZE_DEFINE(CrashDumpRVC2::CrashReport::ErrorSourceInfo::AssertContext)
DEPTHAI_SERIALIZE_DEFINE(CrashDumpRVC2::CrashReport::ErrorSourceInfo::TrapContext)
DEPTHAI_SERIALIZE_DEFINE(CrashDumpRVC2::CrashReport::ErrorSourceInfo)
DEPTHAI_SERIALIZE_DEFINE(CrashDumpRVC2::CrashReport::ThreadCallstack::CallstackContext)
DEPTHAI_SERIALIZE_DEFINE(CrashDumpRVC2::CrashReport::ThreadCallstack)
DEPTHAI_SERIALIZE_DEFINE(CrashDumpRVC2::CrashReport)
DEPTHAI_SERIALIZE_DEFINE(CrashDumpRVC2::CrashReportCollection)

}  // namespace dai
