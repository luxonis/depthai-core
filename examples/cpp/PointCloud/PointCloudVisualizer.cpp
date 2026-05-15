/**
 * PointCloud Visualizer
 *
 * Live 3D visualization of colorized stereo-depth point clouds using the
 * built-in RemoteConnection (foxglove) visualizer.
 *
 * Run the example, then open the visualizer in a browser (default
 * http://localhost:8082) to see the live point cloud.
 * Press 'q' in the viewer or Ctrl-C in the terminal to quit.
 */

#include <csignal>
#include <iostream>

#include "depthai/depthai.hpp"
#include "depthai/remote_connection/RemoteConnection.hpp"

// ---------------------------------------------------------------------------
static volatile std::sig_atomic_t isRunning{1};
void signalHandler(int) {
    isRunning = 0;
}

// ---------------------------------------------------------------------------
int main() {
    std::signal(SIGINT, signalHandler);

    std::cout << "PointCloud Visualizer\n"
              << "=====================\n"
              << "Connecting to device...\n";

    dai::RemoteConnection remoteConnector;
    dai::Pipeline pipeline;

    auto device = pipeline.getDefaultDevice();
    std::cout << "Device: " << device->getDeviceName() << "  (ID: " << device->getDeviceId() << ")\n\n";

    const auto size = std::make_pair(640, 400);

    // ── Cameras ──────────────────────────────────────────────────────
    auto left = pipeline.create<dai::node::Camera>();
    auto right = pipeline.create<dai::node::Camera>();
    auto color = pipeline.create<dai::node::Camera>();

    left->build(dai::CameraBoardSocket::CAM_B);
    right->build(dai::CameraBoardSocket::CAM_C);
    color->build(dai::CameraBoardSocket::CAM_A);

    // ── StereoDepth ──────────────────────────────────────────────────
    auto stereo = pipeline.create<dai::node::StereoDepth>();
    left->requestOutput(size)->link(stereo->left);
    right->requestOutput(size)->link(stereo->right);

    // ── Align depth to color camera ──────────────────────────────────
    auto colorOut = color->requestOutput(size, dai::ImgFrame::Type::RGB888i, dai::ImgResizeMode::CROP, std::nullopt, true);

    auto align = pipeline.create<dai::node::ImageAlign>();
    stereo->depth.link(align->input);
    colorOut->link(align->inputAlignTo);

    // ── PointCloud node ──────────────────────────────────────────────
    auto pc = pipeline.create<dai::node::PointCloud>();
    pc->setRunOnHost(true);
    align->outputAligned.link(pc->inputDepth);
    colorOut->link(pc->getColorInput());

    // Publish the point cloud to the remote visualizer
    remoteConnector.addTopic("pcl", pc->outputPointCloud);

    pipeline.start();
    remoteConnector.registerPipeline(pipeline);

    device->setIrLaserDotProjectorIntensity(0.7);

    std::cout << "Pipeline started.\n"
              << "Open the visualizer at http://localhost:8082 to see the point cloud.\n"
              << "Press 'q' in the viewer or Ctrl-C to quit.\n";

    while(isRunning != 0 && pipeline.isRunning()) {
        int key = remoteConnector.waitKey(1);
        if(key == 'q') {
            std::cout << "Got 'q' key from the remote connection.\n";
            break;
        }
    }

    std::cout << "Done.\n";
    return 0;
}
