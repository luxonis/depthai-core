/**
 * PointCloud Visualizer
 *
 * Live 3D visualization of stereo-depth point clouds using the built-in
 * RemoteConnection (foxglove) visualizer, with a colorized depth preview
 * in an OpenCV window.
 *
 * Run the example, then open the visualizer in a browser (default
 * http://localhost:8082) to see the live point cloud.
 * Press 'q' in viewer / OpenCV window or Ctrl-C in the terminal to quit.
 */

#include <csignal>
#include <iostream>
#include <thread>
#include <chrono>

#include <opencv2/opencv.hpp>
#include "depthai/depthai.hpp"
#include "depthai/remote_connection/RemoteConnection.hpp"

// ---------------------------------------------------------------------------
static std::atomic<bool> isRunning{true};
void signalHandler(int) { isRunning = false; }

// Colorize a uint16 depth frame for display (same logic as the Python version)
static cv::Mat colorizeDepth(const cv::Mat& depthFrame) {
    // Down-sample every 4th row for percentile estimation
    cv::Mat downscaled;
    cv::resize(depthFrame, downscaled, {}, 1.0, 0.25, cv::INTER_NEAREST);

    std::vector<uint16_t> vals;
    vals.reserve(downscaled.total());
    for(int i = 0; i < downscaled.rows; i++) {
        for(int j = 0; j < downscaled.cols; j++) {
            vals.push_back(downscaled.at<uint16_t>(i, j));
        }
    }
    std::sort(vals.begin(), vals.end());

    // Non-zero 1st percentile as min, 99th percentile as max
    auto nonZeroIt = std::upper_bound(vals.begin(), vals.end(), uint16_t(0));
    double minD = (nonZeroIt != vals.end()) ? *std::next(nonZeroIt, std::distance(nonZeroIt, vals.end()) / 100) : 0;
    double maxD = vals[static_cast<size_t>(vals.size() * 0.99)];
    if(maxD <= minD) maxD = minD + 1;

    cv::Mat normalized;
    depthFrame.convertTo(normalized, CV_8UC1, 255.0 / (maxD - minD), -255.0 * minD / (maxD - minD));
    cv::Mat colorized;
    cv::applyColorMap(normalized, colorized, cv::COLORMAP_HOT);
    return colorized;
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
    std::cout << "Device: " << device->getDeviceName()
              << "  (ID: " << device->getDeviceId() << ")\n\n";

    // ── Camera + StereoDepth ─────────────────────────────────────────
    auto left  = pipeline.create<dai::node::Camera>();
    auto right = pipeline.create<dai::node::Camera>();
    auto stereo = pipeline.create<dai::node::StereoDepth>();

    left->build(dai::CameraBoardSocket::CAM_B);
    right->build(dai::CameraBoardSocket::CAM_C);

    left->requestOutput({640, 400})->link(stereo->left);
    right->requestOutput({640, 400})->link(stereo->right);

    // ── PointCloud node ──────────────────────────────────────────────
    auto pc = pipeline.create<dai::node::PointCloud>();
    pc->setRunOnHost(true);
    pc->initialConfig->setLengthUnit(dai::LengthUnit::METER);
    stereo->depth.link(pc->inputDepth);

    // Publish the point cloud to the remote visualizer
    remoteConnector.addTopic("pointcloud", pc->outputPointCloud);

    // Depth output queue for the OpenCV preview
    auto qDepth = stereo->depth.createOutputQueue(4, false);

    pipeline.start();
    remoteConnector.registerPipeline(pipeline);

    std::cout << "Pipeline started.\n"
              << "Open the visualizer at http://localhost:8082 to see the point cloud.\n"
              << "Press 'q' in viewer / OpenCV window or Ctrl-C to quit.\n";

    while(isRunning && pipeline.isRunning()) {
        // Show colorized depth in an OpenCV window
        auto depthMsg = qDepth->tryGet<dai::ImgFrame>();
        if(depthMsg) {
            cv::imshow("Depth", colorizeDepth(depthMsg->getCvFrame()));
        }

        int cvKey = cv::waitKey(1);
        if(cvKey == 'q') break;

        int key = remoteConnector.waitKey(1);
        if(key == 'q') {
            std::cout << "Got 'q' key from the remote connection.\n";
            break;
        }
    }

    cv::destroyAllWindows();

    std::cout << "Done.\n";
    return 0;
}
