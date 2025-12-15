#pragma once

#include "depthai/pipeline/Pipeline.hpp"

namespace dai {
namespace utility {

class PipelineImplHelper {
    PipelineImpl* pipeline;

   public:
    PipelineImplHelper(PipelineImpl* pipeline) : pipeline(pipeline) {};
    void setupHolisticRecordAndReplay();
    void setupPipelineDebuggingPre();
    void setupPipelineDebuggingPost(std::unordered_map<dai::Node::Output*, node::internal::XLinkOutBridge>&,
                                    std::unordered_map<dai::Node::Input*, node::internal::XLinkInBridge>&);
};

}  // namespace utility
}  // namespace dai
