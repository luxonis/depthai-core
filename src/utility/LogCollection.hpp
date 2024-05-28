#pragma once


#include <depthai/pipeline/Pipeline.hpp>


namespace dai {
namespace logCollection {


void logPipeline(const Pipeline& pipeline);

void logPipeline(const PipelineSchema& pipelineSchema);
}  // namespace logCollection
}  // namespace dai
