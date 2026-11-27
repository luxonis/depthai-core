#pragma once

#include "depthai/pipeline/Pipeline.hpp"

namespace dai {
namespace utility {

class PipelineImplHelper {
   public:
    static void setupHolisticRecordAndReplay(const std::weak_ptr<PipelineImpl>& pipelineWeak);
    static void finishHolisticRecordAndReplay(PipelineImpl* pipeline);
    /// Packs a holistic recording into a .tar.gz. Returns false when there is nothing to pack or packing fails.
    static bool packHolisticRecording(PipelineImpl* pipeline);
    static void setupPipelineDebuggingPre(const std::weak_ptr<PipelineImpl>& pipelineWeak);
    static void setupPipelineDebuggingPost(const std::weak_ptr<PipelineImpl>& pipelineWeak,
                                           std::unordered_map<dai::Node::Output*, node::internal::XLinkOutBridge>&,
                                           std::unordered_map<dai::Node::Input*, node::internal::XLinkInBridge>&);
};

}  // namespace utility
}  // namespace dai
