#pragma once

#include <depthai/pipeline/ThreadedHostNode.hpp>
#include <depthai/pipeline/datatype/ImgFrame.hpp>

namespace dai {
namespace node {
/**
 * Display ImgFrame messages in a host OpenCV window with FPS and latency overlays.
 * Pressing 'q' in the window stops the parent pipeline.
 * Requires OpenCV support and a graphical display.
 */
class Display : public dai::NodeCRTP<ThreadedHostNode, Display> {
   private:
    std::string name;

   public:
    /**
     * Create a display node.
     * @param name OpenCV window name, defaulting to "Display".
     */
    explicit Display(std::string name = "Display");
    /// ImgFrame messages to display; frames are converted using ImgFrame::getCvFrame().
    Input input{*this, {}};
    /// Display incoming frames on the pipeline's worker thread until the node stops.
    void run() override;
};
}  // namespace node
}  // namespace dai
