#pragma once


#include <depthai/pipeline/Pipeline.hpp>
#include <depthai/device/CrashDump.hpp>
#include <string>

namespace dai {
namespace logCollection {


void logPipeline(const Pipeline& pipeline);

void logPipeline(const PipelineSchema& pipelineSchema);

void logCrashDump(const std::optional<PipelineSchema>& pipelineSchema, const CrashDump& crashDump);
}  // namespace logCollection
}  // namespace dai
