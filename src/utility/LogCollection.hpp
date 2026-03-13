#pragma once

#include <depthai/device/CrashDump.hpp>
#include <depthai/device/DeviceGate.hpp>
#include <depthai/pipeline/Pipeline.hpp>
#include <variant>

namespace dai {
namespace logCollection {

using GenericCrashDump = std::variant<CrashDump, DeviceGate::CrashDump>;

void logPipeline(const PipelineSchema& pipelineSchema, const dai::DeviceInfo& deviceInfo);

void logCrashDump(const std::optional<PipelineSchema>& pipelineSchema, const GenericCrashDump& crashDump, const dai::DeviceInfo& deviceInfo);
}  // namespace logCollection
}  // namespace dai