#pragma once

#include <depthai/device/CrashDump.hpp>
#include <depthai/pipeline/Pipeline.hpp>

namespace dai {
namespace logCollection {

void logPipeline(const PipelineSchema& pipelineSchema, const dai::DeviceInfo& deviceInfo);

void logCrashDump(const std::optional<PipelineSchema>& pipelineSchema, const CrashDump& crashDump, const dai::DeviceInfo& deviceInfo);
}  // namespace logCollection
}  // namespace dai