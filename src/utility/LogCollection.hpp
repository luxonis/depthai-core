#pragma once

#include <depthai/device/CrashDump.hpp>
#include <depthai/device/DeviceGate.hpp>
#include <depthai/pipeline/Pipeline.hpp>

namespace dai {
namespace logCollection {

void logCrashDump(const std::optional<PipelineSchema>& pipelineSchema, const CrashDump& crashDump, const dai::DeviceInfo& deviceInfo);
}  // namespace logCollection
}  // namespace dai
