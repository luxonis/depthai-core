#pragma once

#include <depthai/pipeline/ThreadedHostNode.hpp>
#include <depthai/pipeline/datatype/ImgFrame.hpp>

namespace dai {
namespace node {
/**
 * Capture images from the host's default OpenCV camera (index 0).
 * Frames are resized to 640x480 and shown in the HostCameraPreview window.
 * Requires OpenCV support, an accessible host camera, and a graphical display.
 */
class HostCamera : public dai::NodeCRTP<ThreadedHostNode, HostCamera> {
   public:
    /// Captured ImgFrame messages with host timestamps and increasing sequence numbers.
    Output out{*this, {DEFAULT_NAME, DEFAULT_GROUP, DEFAULT_TYPES}};
    /// Capture and send frames on the pipeline's worker thread until the node stops.
    void run() override;
};
}  // namespace node
}  // namespace dai
