#include <catch2/catch_all.hpp>
#include <catch2/catch_test_macros.hpp>
#include <cstring>
#include <limits>
#include <depthai/depthai.hpp>

#include "depthai/pipeline/InputQueue.hpp"
#include "depthai/pipeline/MessageQueue.hpp"
#include "depthai/pipeline/datatype/PointCloudData.hpp"

constexpr auto WIDTH = 640;
constexpr auto HEIGHT = 400;
std::shared_ptr<dai::MessageQueue> configurePipeline(bool organized, dai::Pipeline& pipeline) {
    auto monoLeft = pipeline.create<dai::node::Camera>()->build(dai::CameraBoardSocket::CAM_B);
    auto monoRight = pipeline.create<dai::node::Camera>()->build(dai::CameraBoardSocket::CAM_C);
    auto stereo = pipeline.create<dai::node::StereoDepth>();
    auto pointcloud = pipeline.create<dai::node::PointCloud>();

    if(organized) {
        pointcloud->initialConfig->setOrganized(true);
    }

    monoLeft->requestOutput(std::make_pair(WIDTH, HEIGHT))->link(stereo->left);
    monoRight->requestOutput(std::make_pair(WIDTH, HEIGHT))->link(stereo->right);
    stereo->depth.link(pointcloud->inputDepth);
    return pointcloud->outputPointCloud.createOutputQueue();
}

TEST_CASE("organized pointcloud") {
    dai::Pipeline pipeline;
    if(pipeline.getDefaultDevice()->getPlatform() == dai::Platform::RVC2) {
        WARN("Skipping organized pointcloud test: PointCloud node is not supported on RVC2.");
        return;
    }
    auto outQ = configurePipeline(true, pipeline);
    pipeline.start();
    for(int i = 0; i < 10; ++i) {
        auto pcl = outQ->get<dai::PointCloudData>();
        REQUIRE(pcl != nullptr);
        REQUIRE(pcl->getWidth() == WIDTH);
        REQUIRE(pcl->getHeight() == HEIGHT);
        REQUIRE(pcl->getPoints().size() == WIDTH * HEIGHT);
        REQUIRE(pcl->getMinX() <= pcl->getMaxX());
        REQUIRE(pcl->getMinY() <= pcl->getMaxY());
        REQUIRE(pcl->getMinZ() <= pcl->getMaxZ());
    }
}

TEST_CASE("sparse pointcloud") {
    dai::Pipeline pipeline;
    if(pipeline.getDefaultDevice()->getPlatform() == dai::Platform::RVC2) {
        WARN("Skipping sparse pointcloud test: PointCloud node is not supported on RVC2.");
        return;
    }
    auto outQ = configurePipeline(false, pipeline);
    pipeline.start();
    for(int i = 0; i < 10; ++i) {
        auto pcl = outQ->get<dai::PointCloudData>();
        REQUIRE(pcl != nullptr);
        REQUIRE(pcl->getHeight() == 1);
        REQUIRE(pcl->getWidth() == pcl->getPoints().size());
        REQUIRE(pcl->getPoints().size() <= WIDTH * HEIGHT);
        REQUIRE(pcl->getMinX() <= pcl->getMaxX());
        REQUIRE(pcl->getMinY() <= pcl->getMaxY());
        REQUIRE(pcl->getMinZ() <= pcl->getMaxZ());
    }
}

// ============================================================================
// Colorization proceeds with mismatched extrinsics (warn only, no skip)
// ============================================================================
TEST_CASE("Colorization proceeds despite mismatched frame extrinsics") {
    // This test verifies that when depth and color frames have different
    // toCameraSocket extrinsics, the PointCloud node still produces a
    // colorized point cloud (with a warning) rather than falling back to
    // depth-only output.

    dai::Pipeline pipeline;

    auto pc = pipeline.create<dai::node::PointCloud>();
    pc->initialConfig->setLengthUnit(dai::LengthUnit::MILLIMETER);

    auto depthInQ = pc->inputDepth.createInputQueue();
    auto colorInQ = pc->getColorInput().createInputQueue();
    auto outQ = pc->outputPointCloud.createOutputQueue(4, false);

    pipeline.start();

    constexpr unsigned W = 4, H = 4;
    std::array<std::array<float, 3>, 3> intrinsics = {{{100.f, 0.f, 2.f}, {0.f, 100.f, 2.f}, {0.f, 0.f, 1.f}}};

    // Depth extrinsics → CAM_B
    dai::Extrinsics depthExt({{1, 0, 0}, {0, 1, 0}, {0, 0, 1}}, {0, 0, 0}, dai::CameraBoardSocket::CAM_B);
    dai::ImgTransformation depthTransform(W, H, intrinsics, dai::CameraModel::Perspective, {}, depthExt);

    // Color extrinsics → CAM_A (mismatched!)
    dai::Extrinsics colorExt({{1, 0, 0}, {0, 1, 0}, {0, 0, 1}}, {0, 0, 0}, dai::CameraBoardSocket::CAM_A);
    dai::ImgTransformation colorTransform(W, H, intrinsics, dai::CameraModel::Perspective, {}, colorExt);

    // Create synthetic depth frame (RAW16)
    auto depthFrame = std::make_shared<dai::ImgFrame>();
    depthFrame->setWidth(W);
    depthFrame->setHeight(H);
    depthFrame->setType(dai::ImgFrame::Type::RAW16);
    std::vector<uint16_t> depthData(W * H, 1000);
    std::vector<uint8_t> depthBytes(depthData.size() * sizeof(uint16_t));
    std::memcpy(depthBytes.data(), depthData.data(), depthBytes.size());
    depthFrame->setData(std::move(depthBytes));
    depthFrame->setTransformation(depthTransform);

    // Create synthetic color frame (RGB888i) — same size
    auto colorFrame = std::make_shared<dai::ImgFrame>();
    colorFrame->setWidth(W);
    colorFrame->setHeight(H);
    colorFrame->setType(dai::ImgFrame::Type::RGB888i);
    std::vector<uint8_t> colorData(W * H * 3, 0);
    for(unsigned i = 0; i < W * H; ++i) {
        colorData[i * 3 + 0] = 100;  // R
        colorData[i * 3 + 1] = 150;  // G
        colorData[i * 3 + 2] = 200;  // B
    }
    colorFrame->setData(std::move(colorData));
    colorFrame->setTransformation(colorTransform);

    // Send frames
    depthInQ->send(depthFrame);
    colorInQ->send(colorFrame);

    // Get output — should be colorized despite extrinsics mismatch
    auto pcd = outQ->get<dai::PointCloudData>();
    REQUIRE(pcd != nullptr);
    REQUIRE(pcd->isColor());
    REQUIRE(pcd->getWidth() > 0);

    // Verify color data is present
    auto points = pcd->getPointsRGB();
    REQUIRE(!points.empty());
    for(const auto& p : points) {
        if(p.z > 0.f) {
            REQUIRE(p.r == 100);
            REQUIRE(p.g == 150);
            REQUIRE(p.b == 200);
        }
    }

    pipeline.stop();
}

// ============================================================================
// Consistent 3D point: depth-only vs same-sensor color vs cross-sensor aligned
// ============================================================================
TEST_CASE("Consistent 3D point across depth-only, same-sensor color, and cross-sensor aligned") {
    // Three ways to obtain a 3D point from the same depth pixel must agree:
    //   1. Depth frame only → PointCloud
    //   2. Depth + color from the SAME sensor → PointCloud (colorized)
    //   3. Depth aligned (ImageAlign) to a DIFFERENT color sensor → PointCloud
    //      with setTargetCoordinateSystem back to the depth camera
    //
    // A single non-zero pixel at (PX,PY) with depth DEPTH_MM is used.
    // Real intrinsics / extrinsics are read from the connected device.

    dai::Pipeline pipeline;

    // ---- Calibration from device ----
    auto calibHandler = pipeline.getDefaultDevice()->readCalibration();

    constexpr auto DEPTH_SOCKET = dai::CameraBoardSocket::CAM_B;
    constexpr auto COLOR_SOCKET = dai::CameraBoardSocket::CAM_A;

    constexpr unsigned W = 640, H = 400;
    constexpr unsigned PX = 320, PY = 200;
    constexpr unsigned BLOCK_R = 2;  // 5×5 block so ImageAlign interpolation has neighbors
    constexpr uint16_t DEPTH_MM = 2000;  // 2 m

    // Intrinsics at test resolution
    auto depthIntrVec = calibHandler.getCameraIntrinsics(DEPTH_SOCKET, W, H);
    auto colorIntrVec = calibHandler.getCameraIntrinsics(COLOR_SOCKET, W, H);

    auto toArr3x3 = [](const std::vector<std::vector<float>>& v) {
        std::array<std::array<float, 3>, 3> a{};
        for(int i = 0; i < 3; i++)
            for(int j = 0; j < 3; j++)
                a[i][j] = v[i][j];
        return a;
    };
    auto depthIntr = toArr3x3(depthIntrVec);
    auto colorIntr = toArr3x3(colorIntrVec);

    // Extrinsics — all reference COLOR_SOCKET (CAM_A) so that
    // getExtrinsicsTransformationTo() can compute relative poses directly.

    // Depth camera (CAM_B): transform TO CAM_A
    auto B2A = calibHandler.getCameraExtrinsics(DEPTH_SOCKET, COLOR_SOCKET);
    dai::Extrinsics depthExtr(B2A, COLOR_SOCKET);

    // Color camera (CAM_A): identity to self
    dai::Extrinsics colorExtr({{1, 0, 0}, {0, 1, 0}, {0, 0, 1}}, {0, 0, 0}, COLOR_SOCKET);

    // ImgTransformations (zero distortion — sufficient for synthetic data)
    dai::ImgTransformation depthTransform(W, H, depthIntr, dai::CameraModel::Perspective, {}, depthExtr);
    dai::ImgTransformation sameColorTransform(W, H, depthIntr, dai::CameraModel::Perspective, {}, depthExtr);
    dai::ImgTransformation colorTransform(W, H, colorIntr, dai::CameraModel::Perspective, {}, colorExtr);

    // ---- Pipeline: three PointCloud nodes + one ImageAlign ----

    // Case 1: depth only
    auto pc1 = pipeline.create<dai::node::PointCloud>();
    pc1->initialConfig->setLengthUnit(dai::LengthUnit::MILLIMETER);
    pc1->initialConfig->setTargetCoordinateSystem(DEPTH_SOCKET);
    auto depth1Q = pc1->inputDepth.createInputQueue();
    auto out1Q = pc1->outputPointCloud.createOutputQueue(4, false);

    // Case 2: depth + same-sensor color
    auto pc2 = pipeline.create<dai::node::PointCloud>();
    pc2->initialConfig->setLengthUnit(dai::LengthUnit::MILLIMETER);
    pc2->initialConfig->setTargetCoordinateSystem(DEPTH_SOCKET);
    auto depth2Q = pc2->inputDepth.createInputQueue();
    auto color2Q = pc2->getColorInput().createInputQueue();
    auto out2Q = pc2->outputPointCloud.createOutputQueue(4, false);

    // Case 3: depth → ImageAlign → PointCloud with cross-sensor color
    auto align = pipeline.create<dai::node::ImageAlign>();
    align->setRunOnHost(true);
    auto pc3 = pipeline.create<dai::node::PointCloud>();
    pc3->initialConfig->setLengthUnit(dai::LengthUnit::MILLIMETER);
    pc3->initialConfig->setTargetCoordinateSystem(DEPTH_SOCKET);
    auto depthAlignQ = align->input.createInputQueue();
    auto colorAlignQ = align->inputAlignTo.createInputQueue();
    align->outputAligned.link(pc3->inputDepth);
    auto color3Q = pc3->getColorInput().createInputQueue();
    auto out3Q = pc3->outputPointCloud.createOutputQueue(4, false);

    pipeline.start();

    // ---- Helpers: synthetic frame factories ----
    auto makeDepth = [&](const dai::ImgTransformation& t, dai::CameraBoardSocket socket) {
        auto f = std::make_shared<dai::ImgFrame>();
        f->setWidth(W);
        f->setHeight(H);
        f->setType(dai::ImgFrame::Type::RAW16);
        f->setInstanceNum(static_cast<unsigned>(socket));
        std::vector<uint16_t> d(W * H, 0);
        for(unsigned dy = PY - BLOCK_R; dy <= PY + BLOCK_R; dy++)
            for(unsigned dx = PX - BLOCK_R; dx <= PX + BLOCK_R; dx++)
                d[dy * W + dx] = DEPTH_MM;
        std::vector<uint8_t> b(d.size() * sizeof(uint16_t));
        std::memcpy(b.data(), d.data(), b.size());
        f->setData(std::move(b));
        f->setTransformation(t);
        return f;
    };

    auto makeColor = [&](const dai::ImgTransformation& t, dai::CameraBoardSocket socket) {
        auto f = std::make_shared<dai::ImgFrame>();
        f->setWidth(W);
        f->setHeight(H);
        f->setType(dai::ImgFrame::Type::RGB888i);
        f->setInstanceNum(static_cast<unsigned>(socket));
        std::vector<uint8_t> d(W * H * 3);
        for(unsigned i = 0; i < W * H; i++) {
            d[i * 3 + 0] = 100;
            d[i * 3 + 1] = 150;
            d[i * 3 + 2] = 200;
        }
        f->setData(std::move(d));
        f->setTransformation(t);
        return f;
    };

    // ==== Case 1: depth only ====
    depth1Q->send(makeDepth(depthTransform, DEPTH_SOCKET));
    auto pcd1 = out1Q->get<dai::PointCloudData>();
    REQUIRE(pcd1 != nullptr);
    auto pts1 = pcd1->getPoints();
    REQUIRE(pts1.size() == (2 * BLOCK_R + 1) * (2 * BLOCK_R + 1));
    // Pick the center point (corresponding to PX,PY)
    auto point1 = pts1[pts1.size() / 2];
    REQUIRE(point1.z > 0.f);
    INFO("Case 1 point: (" << point1.x << ", " << point1.y << ", " << point1.z << ")");

    // ==== Case 2: depth + same-sensor color ====
    depth2Q->send(makeDepth(depthTransform, DEPTH_SOCKET));
    color2Q->send(makeColor(sameColorTransform, DEPTH_SOCKET));
    auto pcd2 = out2Q->get<dai::PointCloudData>();
    REQUIRE(pcd2 != nullptr);
    REQUIRE(pcd2->isColor());
    auto pts2 = pcd2->getPointsRGB();
    REQUIRE(pts2.size() == (2 * BLOCK_R + 1) * (2 * BLOCK_R + 1));
    auto point2 = pts2[pts2.size() / 2];
    INFO("Case 2 point: (" << point2.x << ", " << point2.y << ", " << point2.z << ")");

    // ==== Case 3: depth → ImageAlign → PointCloud with color ====
    depthAlignQ->send(makeDepth(depthTransform, DEPTH_SOCKET));
    colorAlignQ->send(makeColor(colorTransform, COLOR_SOCKET));
    color3Q->send(makeColor(colorTransform, COLOR_SOCKET));
    auto pcd3 = out3Q->get<dai::PointCloudData>();
    REQUIRE(pcd3 != nullptr);
    REQUIRE(pcd3->isColor());
    auto pts3 = pcd3->getPointsRGB();
    REQUIRE(!pts3.empty());
    INFO("Case 3 num points: " << pts3.size());

    // Find the closest point to point1 (alignment interpolation may spread
    // the single depth pixel across a few output pixels)
    dai::Point3fRGBA point3 = pts3[0];
    float minDist2 = std::numeric_limits<float>::max();
    for(const auto& p : pts3) {
        float d2 = (p.x - point1.x) * (p.x - point1.x)
                 + (p.y - point1.y) * (p.y - point1.y)
                 + (p.z - point1.z) * (p.z - point1.z);
        if(d2 < minDist2) {
            minDist2 = d2;
            point3 = p;
        }
    }
    INFO("Case 3 closest point: (" << point3.x << ", " << point3.y << ", " << point3.z << ")");

    // ---- Verify consistency ----
    constexpr float TOL = 10.f;  // 10 mm — covers interpolation artifacts

    // Cases 1 & 2 must be virtually identical (same intrinsics, same extrinsics)
    REQUIRE(point1.x == Catch::Approx(point2.x).margin(TOL));
    REQUIRE(point1.y == Catch::Approx(point2.y).margin(TOL));
    REQUIRE(point1.z == Catch::Approx(point2.z).margin(TOL));

    // Cases 1 & 3 should agree after coordinate-system transform
    REQUIRE(point1.x == Catch::Approx(point3.x).margin(TOL));
    REQUIRE(point1.y == Catch::Approx(point3.y).margin(TOL));
    REQUIRE(point1.z == Catch::Approx(point3.z).margin(TOL));

    // Verify color data in Case 2
    REQUIRE(point2.r == 100);
    REQUIRE(point2.g == 150);
    REQUIRE(point2.b == 200);

    pipeline.stop();
}
