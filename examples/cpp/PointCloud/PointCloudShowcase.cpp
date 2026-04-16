/**
 * PointCloud Node Showcase
 *
 * Demonstrates filtered/organized output, camera-to-camera transforms,
 * housing coordinate system transforms, and custom 4×4 matrix transforms.
 *
 * See README.md in this directory for full documentation.
 */

#include <chrono>
#include <iomanip>
#include <iostream>
#include <memory>
#include <thread>
#include <vector>

#include "depthai/depthai.hpp"

// ---------------------------------------------------------------------------
// Print helpers
// ---------------------------------------------------------------------------
static void printHeader(const std::string& title) {
    std::cout << "\n╔══════════════════════════════════════════════╗\n";
    std::cout << "║  " << std::left << std::setw(44) << title << "║\n";
    std::cout << "╚══════════════════════════════════════════════╝\n";
}

static void printPointCloudInfo(const dai::PointCloudData& pcd, int frameNum) {
    auto points = pcd.getPoints();

    std::cout << "\n--- Frame " << frameNum << " ---\n";
    std::cout << "  Points       : " << points.size() << "\n";
    std::cout << "  Width×Height : " << pcd.getWidth() << " × " << pcd.getHeight() << "\n";
    std::cout << "  Organized    : " << (pcd.isOrganized() ? "yes" : "no") << "\n";
    std::cout << "  Color        : " << (pcd.isColor() ? "yes" : "no") << "\n";
    std::cout << "  Bounding box :" << "  X [" << pcd.getMinX() << ", " << pcd.getMaxX() << "]" << "  Y [" << pcd.getMinY() << ", " << pcd.getMaxY() << "]"
              << "  Z [" << pcd.getMinZ() << ", " << pcd.getMaxZ() << "]\n";
}

// ---------------------------------------------------------------------------
// Main – single pipeline with multiple PointCloud nodes
// ---------------------------------------------------------------------------
static constexpr int NUM_FRAMES = 3;

int main() {
    std::cout << "PointCloud Node Showcase\n"
              << "========================\n"
              << "Connecting to device...\n";

    auto device = std::make_shared<dai::Device>();
    std::cout << "Device: " << device->getDeviceName() << "  (ID: " << device->getDeviceId() << ")\n";

    try {
        // ==============================================================
        // Single pipeline – shared Camera + StereoDepth, multiple
        // PointCloud nodes configured differently.
        // ==============================================================
        dai::Pipeline pipeline(device);

        auto left = pipeline.create<dai::node::Camera>()->build(dai::CameraBoardSocket::CAM_B);
        auto right = pipeline.create<dai::node::Camera>()->build(dai::CameraBoardSocket::CAM_C);
        auto color = pipeline.create<dai::node::Camera>()->build(dai::CameraBoardSocket::CAM_A);
        auto stereo = pipeline.create<dai::node::StereoDepth>();
        left->requestFullResolutionOutput()->link(stereo->left);
        right->requestFullResolutionOutput()->link(stereo->right);

        // ── 1. Filtered point cloud (METER)
        auto pcSparse = pipeline.create<dai::node::PointCloud>();
        pcSparse->setRunOnHost(true);
        pcSparse->initialConfig->setLengthUnit(dai::LengthUnit::METER);
        stereo->depth.link(pcSparse->inputDepth);
        auto qSparse = pcSparse->outputPointCloud.createOutputQueue();

        // ── 2. Organized point cloud (MILLIMETER)
        auto pcOrganized = pipeline.create<dai::node::PointCloud>();
        pcOrganized->setRunOnHost(true);
        pcOrganized->initialConfig->setLengthUnit(dai::LengthUnit::MILLIMETER);
        pcOrganized->initialConfig->setOrganized(true);
        stereo->depth.link(pcOrganized->inputDepth);
        auto qOrganized = pcOrganized->outputPointCloud.createOutputQueue();

        // ── 3. Transform pointcloud into another camera's coordinate system
        auto pcCam = pipeline.create<dai::node::PointCloud>();
        pcCam->setRunOnHost(true);
        pcCam->initialConfig->setLengthUnit(dai::LengthUnit::MILLIMETER);
        pcCam->initialConfig->setTargetCoordinateSystem(dai::CameraBoardSocket::CAM_A);
        // Or transform to a housing coordinate system instead, e.g.:
        // pcCam->initialConfig->setTargetCoordinateSystem(dai::HousingCoordinateSystem::VESA_A);
        stereo->depth.link(pcCam->inputDepth);
        auto qCam = pcCam->outputPointCloud.createOutputQueue();

        // ── 4. Custom 4×4 transform (90° Z rotation) + passthrough
        auto pcCustom = pipeline.create<dai::node::PointCloud>();
        pcCustom->setRunOnHost(true);
        pcCustom->initialConfig->setLengthUnit(dai::LengthUnit::MILLIMETER);
        pcCustom->useCPU();
        std::array<std::array<float, 4>, 4> transform = {{{{0.f, -1.f, 0.f, 0.f}}, {{1.f, 0.f, 0.f, 0.f}}, {{0.f, 0.f, 1.f, 0.f}}, {{0.f, 0.f, 0.f, 1.f}}}};
        pcCustom->initialConfig->setTransformationMatrix(transform);
        stereo->depth.link(pcCustom->inputDepth);
        auto qCustom = pcCustom->outputPointCloud.createOutputQueue();
        auto qDepth = pcCustom->passthroughDepth.createOutputQueue();

        // ── 5. Colorized point cloud (aligned RGB from color camera)
        auto pcColor = pipeline.create<dai::node::PointCloud>();
        pcColor->setRunOnHost(true);
        pcColor->initialConfig->setLengthUnit(dai::LengthUnit::METER);
        auto* colorOut = color->requestOutput(std::make_pair(640, 400), dai::ImgFrame::Type::RGB888i, dai::ImgResizeMode::CROP, std::nullopt, true);
        auto platform = device->getPlatform();
        if(platform == dai::Platform::RVC4) {
            auto imageAlign = pipeline.create<dai::node::ImageAlign>();
            stereo->depth.link(imageAlign->input);
            colorOut->link(imageAlign->inputAlignTo);
            imageAlign->outputAligned.link(pcColor->inputDepth);
        } else {
            colorOut->link(stereo->inputAlignTo);
            stereo->depth.link(pcColor->inputDepth);
        }
        colorOut->link(pcColor->getColorInput());
        auto qColor = pcColor->outputPointCloud.createOutputQueue();

        // ==============================================================
        // Collect frames – drain all queues evenly to avoid back-pressure
        // ==============================================================
        struct TestCase {
            std::shared_ptr<dai::MessageQueue> queue;
            std::string title;
            std::string config;
            std::vector<std::shared_ptr<dai::PointCloudData>> frames;
        };

        std::vector<TestCase> testCases = {
            {qSparse, "1. Basic sparse point cloud", "METER", {}},
            {qOrganized, "2. Organized point cloud", "MILLIMETER, initialConfig->setOrganized(true)", {}},
            {qCam, "3. Camera-to-camera transform", "setTargetCoordinateSystem(CAM_A)", {}},
            {qCustom, "4. Custom transform matrix + passthrough", "90° Z rotation via initialConfig", {}},
            {qColor, "5. Colorized point cloud (RGB)", "METER, aligned color camera linked to inputColor", {}},
        };

        std::vector<std::shared_ptr<dai::ImgFrame>> depthFrames;

        pipeline.start();

        // Wait for auto-exposure to settle and stereo depth to stabilize
        std::cout << "Waiting for auto-exposure to settle...\n";
        std::this_thread::sleep_for(std::chrono::seconds(1));

        // Drain stale frames that arrived during warm-up
        for(auto& tc : testCases) {
            tc.queue->tryGetAll<dai::PointCloudData>();
        }
        qDepth->tryGetAll<dai::ImgFrame>();

        for(int i = 0; i < NUM_FRAMES; ++i) {
            for(auto& tc : testCases) {
                tc.frames.push_back(tc.queue->get<dai::PointCloudData>());
            }
            depthFrames.push_back(qDepth->get<dai::ImgFrame>());
        }
        pipeline.stop();

        // ==============================================================
        // Display results grouped by feature
        // ==============================================================
        for(const auto& tc : testCases) {
            printHeader(tc.title);
            std::cout << "  Config: " << tc.config << "\n";
            for(int i = 0; i < NUM_FRAMES; ++i) {
                printPointCloudInfo(*tc.frames[i], i);
                // Show depth passthrough info for the custom transform case
                if(tc.title.find("Custom") != std::string::npos) {
                    std::cout << "  Depth frame  : " << depthFrames[i]->getWidth() << " × " << depthFrames[i]->getHeight() << "\n";
                }
            }
        }

    } catch(const std::exception& e) {
        std::cerr << "\nError: " << e.what() << std::endl;
        return 1;
    }

    std::cout << "\nAll demos completed.\n";
    return 0;
}
