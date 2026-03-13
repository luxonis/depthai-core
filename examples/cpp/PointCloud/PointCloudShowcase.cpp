/**
 * PointCloud Node Showcase
 *
 * Demonstrates all capabilities of the PointCloud node using a single pipeline
 * with multiple PointCloud nodes, each configured differently.  Stereo depth
 * fans out to all of them simultaneously.
 *
 * Features shown:
 *   1. Sparse point cloud       – METER, via initialConfig->setLengthUnit()
 *   2. Organized point cloud    – MILLIMETER, via initialConfig->setOrganized(true)
 *   3. Camera-to-camera transform – setTargetCoordinateSystem(CameraBoardSocket)
 *   4. Custom 4×4 transform     – 90° Z rotation via initialConfig + passthrough
 *
 *
 * Coordinate-system transforms
 * ----------------------------
 * The PointCloud node can transform output points into three kinds of
 * coordinate frames via setTargetCoordinateSystem():
 *
 *   A) Camera board socket – uses extrinsic calibration between cameras.
 *      pc->setTargetCoordinateSystem(CameraBoardSocket::<SOCKET>);
 *
 *      Available sockets:
 *        CAM_A, CAM_B, CAM_C, CAM_D, CAM_E, CAM_F, CAM_G, CAM_H,
 *        CAM_I, CAM_J
 *
 *      Optional flag useSpecTranslation (default false):
 *        When true the node uses nominal/spec translation between cameras
 *        instead of per-unit calibration data.
 *
 *   B) Housing coordinate system – uses housing calibration stored on the
 *      device (position of mounting points, IMU, camera fronts, etc.).
 *      pc->setTargetCoordinateSystem(HousingCoordinateSystem::<CS>);
 *
 *      Available coordinate systems:
 *        CAM_A … CAM_J        – camera CS corrected for the housing
 *        FRONT_CAM_A … FRONT_CAM_J – CSs in front of each camera
 *        VESA_A … VESA_J      – VESA mounting-point CSs
 *        IMU                  – IMU sensor CS
 *
 *      Optional flag useSpecTranslation (default true):
 *        When true the node uses nominal/spec housing coordinates;
 *        when false it uses per-unit calibrated values.
 *
 *   C) Custom 4×4 matrix – applied via initialConfig or the runtime
 *      inputConfig queue.
 *      pc->initialConfig->setTransformationMatrix(matrix);
 *      This can also be combined with (A) or (B); the custom matrix is
 *      applied *after* the calibration-based transform.
 */

#include <iostream>
#include <iomanip>
#include <vector>
#include <memory>
#include <thread>
#include <chrono>

#include "depthai/depthai.hpp"

// ---------------------------------------------------------------------------
// Print helpers
// ---------------------------------------------------------------------------
static void printHeader(const std::string& title) {
    std::cout << "\n╔══════════════════════════════════════════════╗\n";
    std::cout << "║  " << std::left << std::setw(44) << title << "║\n";
    std::cout << "╚══════════════════════════════════════════════╝\n";
}

static void printPointCloudInfo(dai::PointCloudData& pcd, int frameNum) {
    auto points = pcd.getPoints();

    std::cout << "\n--- Frame " << frameNum << " ---\n";
    std::cout << "  Points       : " << points.size() << "\n";
    std::cout << "  Width×Height : " << pcd.getWidth() << " × " << pcd.getHeight() << "\n";
    std::cout << "  Organized    : " << (pcd.isOrganized() ? "yes" : "no") << "\n";
    std::cout << "  Color        : " << (pcd.isColor() ? "yes" : "no") << "\n";
    std::cout << "  Bounding box :"
              << "  X [" << pcd.getMinX() << ", " << pcd.getMaxX() << "]"
              << "  Y [" << pcd.getMinY() << ", " << pcd.getMaxY() << "]"
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
    std::cout << "Device: " << device->getDeviceName()
              << "  (ID: " << device->getDeviceId() << ")\n";

    try {
        // ==============================================================
        // Single pipeline – shared Camera + StereoDepth, multiple
        // PointCloud nodes configured differently.
        // ==============================================================
        dai::Pipeline pipeline(device);

        auto left   = pipeline.create<dai::node::Camera>()->build(dai::CameraBoardSocket::CAM_B);
        auto right  = pipeline.create<dai::node::Camera>()->build(dai::CameraBoardSocket::CAM_C);
        auto stereo = pipeline.create<dai::node::StereoDepth>();
        left->requestFullResolutionOutput()->link(stereo->left);
        right->requestFullResolutionOutput()->link(stereo->right);

        // ── 1. Sparse point cloud (METER, multi-threaded, large pool)
        auto pcSparse = pipeline.create<dai::node::PointCloud>();
        pcSparse->setRunOnHost(true);
        pcSparse->initialConfig->setLengthUnit(dai::LengthUnit::METER);
        stereo->depth.link(pcSparse->inputDepth);
        auto qSparse = pcSparse->outputPointCloud.createOutputQueue();

        // ── 2. Organized point cloud (MILLIMETER, single-threaded)
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
        pcCam->setTargetCoordinateSystem(dai::CameraBoardSocket::CAM_A);
        // Or transform to a housing coordinate system instead, e.g.:
        // pcCam->setTargetCoordinateSystem(dai::HousingCoordinateSystem::VESA_A);
        stereo->depth.link(pcCam->inputDepth);
        auto qCam = pcCam->outputPointCloud.createOutputQueue();

        // ── 4. Custom 4×4 transform (90° Z rotation) + passthrough
        auto pcCustom = pipeline.create<dai::node::PointCloud>();
        pcCustom->setRunOnHost(true);
        pcCustom->initialConfig->setLengthUnit(dai::LengthUnit::MILLIMETER);
        pcCustom->useCPU();
        std::array<std::array<float, 4>, 4> transform = {{
            {{ 0.f, -1.f,  0.f,  0.f}},
            {{ 1.f,  0.f,  0.f,  0.f}},
            {{ 0.f,  0.f,  1.f,  0.f}},
            {{ 0.f,  0.f,  0.f,  1.f}}
        }};
        pcCustom->initialConfig->setTransformationMatrix(transform);
        stereo->depth.link(pcCustom->inputDepth);
        auto qCustom = pcCustom->outputPointCloud.createOutputQueue();
        auto qDepth  = pcCustom->passthroughDepth.createOutputQueue();

        // Note: Housing coordinate system transform is also available, e.g.:
        //   pc->setTargetCoordinateSystem(dai::HousingCoordinateSystem::VESA_A);
        // See the file-level comment at the top for all available
        // CameraBoardSocket and HousingCoordinateSystem values.

        // ==============================================================
        // Collect frames – drain all queues evenly to avoid back-pressure
        // ==============================================================
        std::vector<std::shared_ptr<dai::PointCloudData>> sparseFrames, organizedFrames, camFrames, customFrames;
        std::vector<std::shared_ptr<dai::ImgFrame>> depthFrames;

        pipeline.start();

        // Wait for auto-exposure to settle and stereo depth to stabilize
        std::cout << "Waiting for auto-exposure to settle...\n";
        std::this_thread::sleep_for(std::chrono::seconds(1));

        // Drain stale frames that arrived during warm-up
        while(qSparse->tryGet<dai::PointCloudData>()) {}
        while(qOrganized->tryGet<dai::PointCloudData>()) {}
        while(qCam->tryGet<dai::PointCloudData>()) {}
        while(qCustom->tryGet<dai::PointCloudData>()) {}
        while(qDepth->tryGet<dai::ImgFrame>()) {}

        for(int i = 0; i < NUM_FRAMES; ++i) {
            sparseFrames.push_back(qSparse->get<dai::PointCloudData>());
            organizedFrames.push_back(qOrganized->get<dai::PointCloudData>());
            camFrames.push_back(qCam->get<dai::PointCloudData>());
            customFrames.push_back(qCustom->get<dai::PointCloudData>());
            depthFrames.push_back(qDepth->get<dai::ImgFrame>());
        }
        pipeline.stop();

        // ==============================================================
        // Display results grouped by feature
        // ==============================================================

        // 1 ── Sparse point cloud
        printHeader("1. Basic sparse point cloud");
        std::cout << "  Config: METER\n";
        for(int i = 0; i < NUM_FRAMES; ++i)
            printPointCloudInfo(*sparseFrames[i], i);

        // 2 ── Organized point cloud
        printHeader("2. Organized point cloud");
        std::cout << "  Config: MILLIMETER, initialConfig->setOrganized(true)\n";
        for(int i = 0; i < NUM_FRAMES; ++i)
            printPointCloudInfo(*organizedFrames[i], i);

        // 3 ── Transform pointcloud into another camera's coordinate system
        printHeader("3. Camera-to-camera transform");
        std::cout << "  Config: setTargetCoordinateSystem(CAM_A)\n";
        for(int i = 0; i < NUM_FRAMES; ++i)
            printPointCloudInfo(*camFrames[i], i);

        // 4 ── Custom transform + passthrough depth
        printHeader("4. Custom transform matrix + passthrough");
        std::cout << "  Config: 90° Z rotation via initialConfig\n";
        for(int i = 0; i < NUM_FRAMES; ++i) {
            printPointCloudInfo(*customFrames[i], i);
            std::cout << "  Depth frame  : " << depthFrames[i]->getWidth()
                      << " × " << depthFrames[i]->getHeight() << "\n";
        }

    } catch(const std::exception& e) {
        std::cerr << "\nError: " << e.what() << std::endl;
        return 1;
    }

    std::cout << "\nAll demos completed.\n";
    return 0;
}
