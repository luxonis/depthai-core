#pragma once

namespace dai {
namespace utility {

/**
 * Marks the calling thread as a thread that pipeline teardown joins (node threads, device threads),
 * for as long as the scope exists.
 *
 * Such a thread can hold the last reference to a pipeline, for example through Node::getParentPipeline().
 * The pipeline destructor stops the pipeline and joins these threads, so it must not run on one of them.
 * When the last reference is released on a marked thread, the pipeline is destroyed on a separate thread.
 */
class JoinedByPipelineScope {
   public:
    JoinedByPipelineScope();
    ~JoinedByPipelineScope();
    JoinedByPipelineScope(const JoinedByPipelineScope&) = delete;
    JoinedByPipelineScope& operator=(const JoinedByPipelineScope&) = delete;
    JoinedByPipelineScope(JoinedByPipelineScope&&) = delete;
    JoinedByPipelineScope& operator=(JoinedByPipelineScope&&) = delete;

    /// True when the calling thread is inside a JoinedByPipelineScope
    static bool isActive();
};

}  // namespace utility
}  // namespace dai
