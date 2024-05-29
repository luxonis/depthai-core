#pragma once


#include <depthai/pipeline/Pipeline.hpp>
#include <depthai/device/CrashDump.hpp>
#include <string>

namespace dai {
namespace logCollection {


void logPipeline(const Pipeline& pipeline, const dai::DeviceInfo& deviceInfo);

void logPipeline(const PipelineSchema& pipelineSchema, const dai::DeviceInfo& deviceInfo);

void logCrashDump(const std::optional<PipelineSchema>& pipelineSchema, const CrashDump& crashDump, const dai::DeviceInfo& deviceInfo);
}  // namespace logCollection
}  // namespace dai
