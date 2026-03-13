#include <algorithm>
#include <catch2/catch_all.hpp>
#include <cstdint>
#include <cstring>
#include <vector>

#include "depthai/common/CameraBoardSocket.hpp"
#include "depthai/common/DepthUnit.hpp"
#include "depthai/common/EepromData.hpp"
#include "depthai/common/Extrinsics.hpp"
#include "depthai/common/HousingCoordinateSystem.hpp"
#include "depthai/common/Point3f.hpp"
#include "depthai/device/CalibrationHandler.hpp"
#include "depthai/pipeline/datatype/PointCloudData.hpp"
#include "depthai/pipeline/node/PointCloud.hpp"


// ============================================================================
// Helpers: synthetic depth images
// ============================================================================
namespace {

/// Create a uint16 depth image filled with a constant depth (in mm).
std::vector<uint16_t> makeConstantDepth(unsigned int w, unsigned int h, uint16_t depthMm) {
    return std::vector<uint16_t>(w * h, depthMm);
}

/// Depth image with every Nth pixel set to 0 (invalid).
std::vector<uint16_t> makeDepthWithHoles(unsigned int w, unsigned int h, uint16_t depthMm, unsigned int holeEveryN) {
    std::vector<uint16_t> img(w * h);
    for(unsigned int i = 0; i < w * h; ++i) {
        img[i] = (i % holeEveryN == 0) ? 0 : depthMm;
    }
    return img;
}

/// Depth gradient: depth increases linearly along rows.
std::vector<uint16_t> makeDepthGradient(unsigned int w, unsigned int h, uint16_t minDepth, uint16_t maxDepth) {
    std::vector<uint16_t> img(w * h);
    for(unsigned int row = 0; row < h; ++row) {
        uint16_t d = static_cast<uint16_t>(minDepth + (maxDepth - minDepth) * row / (h - 1));
        for(unsigned int col = 0; col < w; ++col) {
            img[row * w + col] = d;
        }
    }
    return img;
}

const uint8_t* asBytes(const std::vector<uint16_t>& v) {
    return reinterpret_cast<const uint8_t*>(v.data());
}

/// Convenience: run Impl dense compute on a synthetic depth image.
std::vector<dai::Point3f> computeDense(dai::node::PointCloud::Impl& impl, const std::vector<uint16_t>& depthImg) {
    std::vector<dai::Point3f> pts;
    impl.computePointCloudDense(asBytes(depthImg), pts);
    return pts;
}

/// Identity 3×3 rotation matrix.
std::vector<std::vector<float>> eye3() {
    return {{1, 0, 0}, {0, 1, 0}, {0, 0, 1}};
}

/// 90° rotation around the Z axis.
std::vector<std::vector<float>> rot90z() {
    return {{0, -1, 0}, {1, 0, 0}, {0, 0, 1}};
}

/// 90° rotation around the Y axis.
std::vector<std::vector<float>> rot90y() {
    return {{0, 0, 1}, {0, 1, 0}, {-1, 0, 0}};
}

/// Standard intrinsics used in CalibrationHandler tests.
std::vector<std::vector<float>> defaultIntrinsics() {
    return {{500.f, 0.f, 320.f}, {0.f, 500.f, 240.f}, {0.f, 0.f, 1.f}};
}

/// Build a CalibrationHandler with two cameras on a straight chain.
///   CAM_B  --(R, T_cm, specT_cm)--> CAM_C  --> AUTO (origin)
/// Returns the handler ready for getCameraExtrinsics().
dai::CalibrationHandler makeTwoCameraHandler(const std::vector<std::vector<float>>& rotation,
                                             const std::vector<float>& translationCm,
                                             const std::vector<float>& specTranslationCm = {0, 0, 0}) {
    dai::CalibrationHandler handler;
    auto intr = defaultIntrinsics();
    handler.setCameraIntrinsics(dai::CameraBoardSocket::CAM_B, intr, 640, 480);
    handler.setCameraIntrinsics(dai::CameraBoardSocket::CAM_C, intr, 640, 480);
    handler.setCameraExtrinsics(dai::CameraBoardSocket::CAM_B, dai::CameraBoardSocket::CAM_C, rotation, translationCm, specTranslationCm);
    return handler;
}

/// Build a CalibrationHandler with three cameras on a chain.
///   CAM_A  --(R_AB, T_AB)--> CAM_B  --(R_BC, T_BC)--> CAM_C  --> AUTO (origin)
dai::CalibrationHandler makeThreeCameraHandler(const std::vector<std::vector<float>>& R_AB,
                                               const std::vector<float>& T_AB_cm,
                                               const std::vector<std::vector<float>>& R_BC,
                                               const std::vector<float>& T_BC_cm) {
    dai::CalibrationHandler handler;
    auto intr = defaultIntrinsics();
    handler.setCameraIntrinsics(dai::CameraBoardSocket::CAM_A, intr, 640, 480);
    handler.setCameraIntrinsics(dai::CameraBoardSocket::CAM_B, intr, 640, 480);
    handler.setCameraIntrinsics(dai::CameraBoardSocket::CAM_C, intr, 640, 480);
    handler.setCameraExtrinsics(dai::CameraBoardSocket::CAM_B, dai::CameraBoardSocket::CAM_C, R_BC, T_BC_cm);
    handler.setCameraExtrinsics(dai::CameraBoardSocket::CAM_A, dai::CameraBoardSocket::CAM_B, R_AB, T_AB_cm);
    return handler;
}

/// Build a CalibrationHandler with housing extrinsics.
///   housingExtrinsics links to `housingOriginCamera` with given rotation & translation (in cm).
///   Camera setup: CAM_B --(R, T)--> CAM_C --> AUTO (origin), or simpler variants.
dai::CalibrationHandler makeHousingHandler(dai::CameraBoardSocket housingOriginCamera,
                                           const std::vector<std::vector<float>>& housingRot,
                                           const dai::Point3f& housingTransCm,
                                           const dai::Point3f& housingSpecTransCm = {0, 0, 0}) {
    dai::EepromData eeprom;
    // Set housing extrinsics
    eeprom.housingExtrinsics.rotationMatrix = housingRot;
    eeprom.housingExtrinsics.translation = housingTransCm;
    eeprom.housingExtrinsics.specTranslation = housingSpecTransCm;
    eeprom.housingExtrinsics.toCameraSocket = housingOriginCamera;

    dai::CalibrationHandler handler(eeprom);
    auto intr = defaultIntrinsics();
    handler.setCameraIntrinsics(dai::CameraBoardSocket::CAM_B, intr, 640, 480);
    handler.setCameraIntrinsics(dai::CameraBoardSocket::CAM_C, intr, 640, 480);
    // CAM_B → CAM_C chain (identity extrinsics if we only need CAM_B as src)
    return handler;
}

}  // namespace

// ============================================================================
// Constant-depth image → flat point cloud at known Z
// ============================================================================
TEST_CASE("Constant depth produces flat point cloud at expected Z", "[PointCloud][Impl]") {
    dai::node::PointCloud::Impl impl;
    constexpr unsigned W = 64, H = 48;
    constexpr float fx = 200.f, fy = 200.f, cx = 32.f, cy = 24.f;
    impl.setIntrinsics(fx, fy, cx, cy, W, H);

    auto depth = makeConstantDepth(W, H, 1000);  // 1 m in mm
    auto pts = computeDense(impl, depth);

    REQUIRE(pts.size() == W * H);

    // Every point must have z = 1000 mm (default unit = mm, scale = 1)
    for(const auto& p : pts) {
        REQUIRE(p.z == Catch::Approx(1000.f));
    }

    // Centre pixel (cx, cy) should map to x=0, y=0
    auto& centre = pts[static_cast<size_t>(cy) * W + static_cast<size_t>(cx)];
    REQUIRE(centre.x == Catch::Approx(0.f));
    REQUIRE(centre.y == Catch::Approx(0.f));

    // Top-left corner: x = (0 - cx)*z/fx = -32*1000/200 = -160
    auto& tl = pts[0];
    REQUIRE(tl.x == Catch::Approx(-160.f));
    REQUIRE(tl.y == Catch::Approx(-120.f));
}

// ============================================================================
// Zero-depth pixels produce (0, 0, 0)
// ============================================================================
TEST_CASE("Zero depth pixels produce zero-valued points", "[PointCloud][Impl]") {
    dai::node::PointCloud::Impl impl;
    constexpr unsigned W = 4, H = 4;
    impl.setIntrinsics(100.f, 100.f, 2.f, 2.f, W, H);

    auto depth = makeConstantDepth(W, H, 0);
    auto pts = computeDense(impl, depth);

    for(const auto& p : pts) {
        REQUIRE(p.x == 0.f);
        REQUIRE(p.y == 0.f);
        REQUIRE(p.z == 0.f);
    }
}

// ============================================================================
// Depth with holes → filterValidPoints keeps only z > 0
// ============================================================================
TEST_CASE("filterValidPoints removes all zero-depth entries", "[PointCloud][Impl]") {
    dai::node::PointCloud::Impl impl;
    constexpr unsigned W = 16, H = 16;
    impl.setIntrinsics(300.f, 300.f, 8.f, 8.f, W, H);

    auto depth = makeDepthWithHoles(W, H, 2000, 4);  // every 4th pixel = 0
    auto dense = computeDense(impl, depth);

    REQUIRE(dense.size() == W * H);

    auto sparse = impl.filterValidPoints(dense);

    // Count expected valid pixels
    size_t expectedValid = 0;
    for(auto d : depth)
        if(d > 0) ++expectedValid;

    REQUIRE(sparse.size() == expectedValid);

    for(const auto& p : sparse) {
        REQUIRE(p.z > 0.f);
    }
}

// ============================================================================
// Length-unit scaling: mm (default), meters, centimeters
// ============================================================================
TEST_CASE("Length unit changes scale Z values correctly", "[PointCloud][Impl][LengthUnit]") {
    constexpr unsigned W = 2, H = 2;
    constexpr uint16_t DEPTH_MM = 5000;  // 5 m

    auto depth = makeConstantDepth(W, H, DEPTH_MM);

    SECTION("Default (mm): z == 5000") {
        dai::node::PointCloud::Impl impl;
        impl.setIntrinsics(100.f, 100.f, 1.f, 1.f, W, H);
        auto pts = computeDense(impl, depth);
        REQUIRE(pts[0].z == Catch::Approx(5000.f));
    }
    SECTION("Meters: z == 5") {
        dai::node::PointCloud::Impl impl;
        impl.setIntrinsics(100.f, 100.f, 1.f, 1.f, W, H);
        impl.setLengthUnit(dai::LengthUnit::METER);
        auto pts = computeDense(impl, depth);
        REQUIRE(pts[0].z == Catch::Approx(5.f));
    }
    SECTION("Centimeters: z == 500") {
        dai::node::PointCloud::Impl impl;
        impl.setIntrinsics(100.f, 100.f, 1.f, 1.f, W, H);
        impl.setLengthUnit(dai::LengthUnit::CENTIMETER);
        auto pts = computeDense(impl, depth);
        REQUIRE(pts[0].z == Catch::Approx(500.f));
    }
    SECTION("Feet: z ≈ 16.4042") {
        dai::node::PointCloud::Impl impl;
        impl.setIntrinsics(100.f, 100.f, 1.f, 1.f, W, H);
        impl.setLengthUnit(dai::LengthUnit::FOOT);
        auto pts = computeDense(impl, depth);
        REQUIRE(pts[0].z == Catch::Approx(16.4042f).epsilon(0.01));
    }
    SECTION("Inches: z ≈ 196.8505") {
        dai::node::PointCloud::Impl impl;
        impl.setIntrinsics(100.f, 100.f, 1.f, 1.f, W, H);
        impl.setLengthUnit(dai::LengthUnit::INCH);
        auto pts = computeDense(impl, depth);
        REQUIRE(pts[0].z == Catch::Approx(196.8505f).epsilon(0.01));
    }
}

// ============================================================================
// CPU and CPU_MT produce identical results
// ============================================================================
TEST_CASE("CPU and CPU_MT produce identical point clouds", "[PointCloud][Impl][MT]") {
    constexpr unsigned W = 32, H = 24;
    constexpr float fx = 300.f, fy = 300.f, cx = 16.f, cy = 12.f;

    // Interesting depth data (varying per pixel)
    std::vector<uint16_t> depth(W * H);
    for(unsigned i = 0; i < W * H; ++i) depth[i] = static_cast<uint16_t>(500 + (i * 7) % 1500);

    dai::node::PointCloud::Impl implST;
    implST.setIntrinsics(fx, fy, cx, cy, W, H);
    implST.useCPU();
    auto ptsST = computeDense(implST, depth);

    for(uint32_t threads : {2, 4, 8}) {
        dai::node::PointCloud::Impl implMT;
        implMT.setIntrinsics(fx, fy, cx, cy, W, H);
        implMT.useCPUMT(threads);
        auto ptsMT = computeDense(implMT, depth);

        REQUIRE(ptsST.size() == ptsMT.size());
        for(size_t i = 0; i < ptsST.size(); ++i) {
            REQUIRE(ptsST[i].x == Catch::Approx(ptsMT[i].x));
            REQUIRE(ptsST[i].y == Catch::Approx(ptsMT[i].y));
            REQUIRE(ptsST[i].z == Catch::Approx(ptsMT[i].z));
        }
    }
}

// ============================================================================
// Depth gradient → monotonically increasing Z
// ============================================================================
TEST_CASE("Depth gradient produces monotonically increasing Z per row", "[PointCloud][Impl]") {
    dai::node::PointCloud::Impl impl;
    constexpr unsigned W = 8, H = 32;
    impl.setIntrinsics(200.f, 200.f, 4.f, 16.f, W, H);

    auto depth = makeDepthGradient(W, H, 500, 5000);
    auto pts = computeDense(impl, depth);

    // Centre column (col = 4) – Z should increase row by row
    for(unsigned row = 1; row < H; ++row) {
        float prevZ = pts[(row - 1) * W + 4].z;
        float curZ = pts[row * W + 4].z;
        REQUIRE(curZ >= prevZ);
    }
}

// ============================================================================
// Identity extrinsics leave points unchanged
// ============================================================================
TEST_CASE("Identity extrinsics do not change points", "[PointCloud][Impl][Transform]") {
    dai::node::PointCloud::Impl impl;
    constexpr unsigned W = 4, H = 4;
    impl.setIntrinsics(100.f, 100.f, 2.f, 2.f, W, H);

    auto depth = makeConstantDepth(W, H, 1000);
    auto pts = computeDense(impl, depth);

    // Copy before transform
    auto before = pts;

    impl.setExtrinsics({{1, 0, 0, 0}, {0, 1, 0, 0}, {0, 0, 1, 0}, {0, 0, 0, 1}});
    impl.applyTransformation(pts);

    for(size_t i = 0; i < pts.size(); ++i) {
        REQUIRE(pts[i].x == Catch::Approx(before[i].x));
        REQUIRE(pts[i].y == Catch::Approx(before[i].y));
        REQUIRE(pts[i].z == Catch::Approx(before[i].z));
    }
}

// ============================================================================
// Pure translation shifts all valid points
// ============================================================================
TEST_CASE("Translation extrinsics shift valid points", "[PointCloud][Impl][Transform]") {
    dai::node::PointCloud::Impl impl;
    constexpr unsigned W = 4, H = 4;
    impl.setIntrinsics(100.f, 100.f, 2.f, 2.f, W, H);

    auto depth = makeConstantDepth(W, H, 2000);
    auto pts = computeDense(impl, depth);
    auto before = pts;

    impl.setExtrinsics({{1, 0, 0, 10}, {0, 1, 0, 20}, {0, 0, 1, 30}, {0, 0, 0, 1}});
    impl.applyTransformation(pts);

    for(size_t i = 0; i < pts.size(); ++i) {
        if(before[i].z > 0.f) {
            REQUIRE(pts[i].x == Catch::Approx(before[i].x + 10.f));
            REQUIRE(pts[i].y == Catch::Approx(before[i].y + 20.f));
            REQUIRE(pts[i].z == Catch::Approx(before[i].z + 30.f));
        }
    }
}

// ============================================================================
// 90° rotation around Z: (x,y,z) → (−y, x, z)
// ============================================================================
TEST_CASE("90-deg Z rotation transforms correctly", "[PointCloud][Impl][Transform]") {
    dai::node::PointCloud::Impl impl;
    impl.setExtrinsics({{0, -1, 0, 0}, {1, 0, 0, 0}, {0, 0, 1, 0}, {0, 0, 0, 1}});

    std::vector<dai::Point3f> pts = {{3.f, 4.f, 5.f}};
    impl.applyTransformation(pts);

    REQUIRE(pts[0].x == Catch::Approx(-4.f));
    REQUIRE(pts[0].y == Catch::Approx(3.f));
    REQUIRE(pts[0].z == Catch::Approx(5.f));
}

// ============================================================================
// Transform skips invalid (z ≤ 0) points
// ============================================================================
TEST_CASE("Transform skips invalid points (z <= 0)", "[PointCloud][Impl][Transform]") {
    dai::node::PointCloud::Impl impl;
    impl.setExtrinsics({{1, 0, 0, 999}, {0, 1, 0, 999}, {0, 0, 1, 999}, {0, 0, 0, 1}});

    std::vector<dai::Point3f> pts = {{1.f, 2.f, 0.f}, {3.f, 4.f, -1.f}};
    impl.applyTransformation(pts);

    // Both should remain untouched
    REQUIRE(pts[0].x == Catch::Approx(1.f));
    REQUIRE(pts[0].z == Catch::Approx(0.f));
    REQUIRE(pts[1].x == Catch::Approx(3.f));
    REQUIRE(pts[1].z == Catch::Approx(-1.f));
}

// ============================================================================
// setExtrinsics rejects non-4×4 matrices
// ============================================================================
TEST_CASE("setExtrinsics rejects non-4x4 matrices", "[PointCloud][Impl][Transform]") {
    dai::node::PointCloud::Impl impl;
    REQUIRE_THROWS_AS(impl.setExtrinsics({{1, 0, 0}, {0, 1, 0}, {0, 0, 1}}), std::runtime_error);
}

// ============================================================================
// computePointCloudDense throws without intrinsics
// ============================================================================
TEST_CASE("computePointCloudDense throws without intrinsics", "[PointCloud][Impl]") {
    dai::node::PointCloud::Impl impl;
    std::vector<uint16_t> d(4, 1000);
    std::vector<dai::Point3f> pts;
    REQUIRE_THROWS_AS(impl.computePointCloudDense(asBytes(d), pts), std::runtime_error);
}

// ============================================================================
// Full pipeline: depth → dense → transform → filter → PointCloudData
// ============================================================================
TEST_CASE("Synthetic depth → PointCloudData organized output", "[PointCloud][Integration]") {
    dai::node::PointCloud::Impl impl;
    constexpr unsigned W = 32, H = 24;
    constexpr float fx = 300.f, fy = 300.f, cx = 16.f, cy = 12.f;
    impl.setIntrinsics(fx, fy, cx, cy, W, H);
    impl.setLengthUnit(dai::LengthUnit::METER);

    // Translate 0.5 m along X
    impl.setExtrinsics({{1, 0, 0, 0.5f}, {0, 1, 0, 0}, {0, 0, 1, 0}, {0, 0, 0, 1}});

    auto depth = makeDepthWithHoles(W, H, 3000, 5);  // 3 m, every 5th = 0
    auto densePoints = computeDense(impl, depth);
    impl.applyTransformation(densePoints);

    // Organized path: keep all points
    auto pcd = std::make_shared<dai::PointCloudData>();
    pcd->setPoints(densePoints);
    pcd->setWidth(W).setHeight(H);

    REQUIRE(pcd->isOrganized());
    REQUIRE(pcd->getWidth() == W);
    REQUIRE(pcd->getHeight() == H);

    auto out = pcd->getPoints();
    REQUIRE(out.size() == W * H);

    // Compute bounding box ourselves and set it on the message
    float minX = std::numeric_limits<float>::max(), maxX = std::numeric_limits<float>::lowest();
    float minY = minX, maxY = maxX, minZ = minX, maxZ = maxX;
    for(const auto& p : out) {
        if(p.z > 0.f) {
            minX = std::min(minX, p.x);
            maxX = std::max(maxX, p.x);
            minY = std::min(minY, p.y);
            maxY = std::max(maxY, p.y);
            minZ = std::min(minZ, p.z);
            maxZ = std::max(maxZ, p.z);
        }
    }
    pcd->setMinX(minX).setMinY(minY).setMinZ(minZ);
    pcd->setMaxX(maxX).setMaxY(maxY).setMaxZ(maxZ);

    // Translation shifts all valid X by +0.5 m
    REQUIRE(pcd->getMinX() > 0.f);                     // shifted positive
    REQUIRE(pcd->getMinZ() == Catch::Approx(3.f));     // depth 3000 mm → 3 m
    REQUIRE(pcd->getMinZ() <= pcd->getMaxZ());
}

TEST_CASE("Synthetic depth → PointCloudData sparse output", "[PointCloud][Integration]") {
    dai::node::PointCloud::Impl impl;
    constexpr unsigned W = 16, H = 16;
    impl.setIntrinsics(200.f, 200.f, 8.f, 8.f, W, H);

    auto depth = makeDepthWithHoles(W, H, 4000, 3);  // every 3rd = 0
    auto densePoints = computeDense(impl, depth);
    auto sparsePoints = impl.filterValidPoints(densePoints);

    auto pcd = std::make_shared<dai::PointCloudData>();
    pcd->setPoints(sparsePoints);
    pcd->setWidth(static_cast<unsigned>(sparsePoints.size())).setHeight(1);

    REQUIRE_FALSE(pcd->isOrganized());
    REQUIRE(pcd->getHeight() == 1);
    REQUIRE(pcd->getWidth() == sparsePoints.size());

    auto out = pcd->getPoints();
    for(const auto& p : out) {
        REQUIRE(p.z > 0.f);
    }
}

// ============================================================================
// Switching length unit between computations
// ============================================================================
TEST_CASE("Switching length unit between computations", "[PointCloud][Impl][LengthUnit]") {
    dai::node::PointCloud::Impl impl;
    constexpr unsigned W = 2, H = 2;
    impl.setIntrinsics(100.f, 100.f, 1.f, 1.f, W, H);

    auto depth = makeConstantDepth(W, H, 2000);

    // mm
    auto ptsMm = computeDense(impl, depth);
    REQUIRE(ptsMm[0].z == Catch::Approx(2000.f));

    // switch to m
    impl.setLengthUnit(dai::LengthUnit::METER);
    auto ptsM = computeDense(impl, depth);
    REQUIRE(ptsM[0].z == Catch::Approx(2.f));

    // switch back to mm
    impl.setLengthUnit(dai::LengthUnit::MILLIMETER);
    auto ptsMm2 = computeDense(impl, depth);
    REQUIRE(ptsMm2[0].z == Catch::Approx(2000.f));
}

// ============================================================================
// Pinhole geometry symmetry from a synthetic depth image
// ============================================================================
TEST_CASE("Pinhole projection symmetry around principal point", "[PointCloud][Impl][Geometry]") {
    dai::node::PointCloud::Impl impl;
    // Symmetric camera: cx at center, cy at center
    constexpr unsigned W = 10, H = 10;
    constexpr float fx = 50.f, fy = 50.f, cx = 4.5f, cy = 4.5f;
    impl.setIntrinsics(fx, fy, cx, cy, W, H);

    auto depth = makeConstantDepth(W, H, 500);
    auto pts = computeDense(impl, depth);

    // Pixel (0, row) vs pixel (9, row) should have opposite-sign X and same |x|
    // col=0: x = (0 - 4.5)*500/50 = -45;  col=9: x = (9-4.5)*500/50 = +45
    for(unsigned row = 0; row < H; ++row) {
        float xLeft = pts[row * W + 0].x;
        float xRight = pts[row * W + 9].x;
        REQUIRE(xLeft == Catch::Approx(-xRight));
    }
}

// ============================================================================
// Dense → transform → filter end-to-end
// ============================================================================
TEST_CASE("Dense compute, transform, filter end-to-end", "[PointCloud][Impl][Pipeline]") {
    dai::node::PointCloud::Impl impl;
    constexpr unsigned W = 8, H = 8;
    impl.setIntrinsics(200.f, 200.f, 4.f, 4.f, W, H);
    // translate z + 100 mm
    impl.setExtrinsics({{1, 0, 0, 0}, {0, 1, 0, 0}, {0, 0, 1, 100}, {0, 0, 0, 1}});

    auto depth = makeDepthWithHoles(W, H, 1000, 3);
    auto dense = computeDense(impl, depth);
    impl.applyTransformation(dense);
    auto sparse = impl.filterValidPoints(dense);

    // All sparse z must be > 100 (was 1000+100 originally, or just 100 for zero-depth—but those are filtered)
    for(const auto& p : sparse) {
        REQUIRE(p.z > 100.f);
    }
}

// ============================================================================
// Camera-to-camera extrinsics: pure translation
// ============================================================================
TEST_CASE("CalibrationHandler camera-to-camera pure translation", "[PointCloud][CalibrationHandler][CameraExtrinsics]") {
    // CAM_B → CAM_C with identity rotation, 7.5 cm X translation (stored in cm)
    auto handler = makeTwoCameraHandler(eye3(), {7.5f, 0.f, 0.f});

    SECTION("Translation in millimeters") {
        auto mat = handler.getCameraExtrinsics(dai::CameraBoardSocket::CAM_B, dai::CameraBoardSocket::CAM_C, false, dai::LengthUnit::MILLIMETER);
        REQUIRE(mat.size() == 4);
        // Rotation block is identity
        REQUIRE(mat[0][0] == Catch::Approx(1.f));
        REQUIRE(mat[1][1] == Catch::Approx(1.f));
        REQUIRE(mat[2][2] == Catch::Approx(1.f));
        // Translation: 7.5 cm → 75 mm
        REQUIRE(mat[0][3] == Catch::Approx(75.f));
        REQUIRE(mat[1][3] == Catch::Approx(0.f));
        REQUIRE(mat[2][3] == Catch::Approx(0.f));
    }

    SECTION("Translation in meters") {
        auto mat = handler.getCameraExtrinsics(dai::CameraBoardSocket::CAM_B, dai::CameraBoardSocket::CAM_C, false, dai::LengthUnit::METER);
        REQUIRE(mat[0][3] == Catch::Approx(0.075f));
    }

    SECTION("Translation in centimeters") {
        auto mat = handler.getCameraExtrinsics(dai::CameraBoardSocket::CAM_B, dai::CameraBoardSocket::CAM_C, false, dai::LengthUnit::CENTIMETER);
        REQUIRE(mat[0][3] == Catch::Approx(7.5f));
    }
}

// ============================================================================
// Camera-to-camera: apply extrinsics to a point cloud
// ============================================================================
TEST_CASE("Camera-to-camera translation applied to point cloud", "[PointCloud][CalibrationHandler][CameraExtrinsics]") {
    auto handler = makeTwoCameraHandler(eye3(), {7.5f, 0.f, 0.f});  // 7.5 cm = 75 mm

    auto mat = handler.getCameraExtrinsics(dai::CameraBoardSocket::CAM_B, dai::CameraBoardSocket::CAM_C, false, dai::LengthUnit::MILLIMETER);

    dai::node::PointCloud::Impl impl;
    constexpr unsigned W = 4, H = 4;
    impl.setIntrinsics(100.f, 100.f, 2.f, 2.f, W, H);
    impl.setExtrinsics(mat);

    auto depth = makeConstantDepth(W, H, 1000);
    auto pts = computeDense(impl, depth);
    auto before = pts;
    impl.applyTransformation(pts);

    // Every valid point should be shifted by +75 mm in X
    for(size_t i = 0; i < pts.size(); ++i) {
        if(before[i].z > 0.f) {
            REQUIRE(pts[i].x == Catch::Approx(before[i].x + 75.f));
            REQUIRE(pts[i].y == Catch::Approx(before[i].y));
            REQUIRE(pts[i].z == Catch::Approx(before[i].z));
        }
    }
}

// ============================================================================
// Camera-to-camera: 90° rotation around Y + translation
// ============================================================================
TEST_CASE("Camera-to-camera 90-deg Y rotation with translation", "[PointCloud][CalibrationHandler][CameraExtrinsics]") {
    // 90° around Y: x→z, z→-x
    auto handler = makeTwoCameraHandler(rot90y(), {0.f, 0.f, 10.f});  // 10 cm Z translation

    auto mat = handler.getCameraExtrinsics(dai::CameraBoardSocket::CAM_B, dai::CameraBoardSocket::CAM_C, false, dai::LengthUnit::MILLIMETER);

    // Rotation block
    REQUIRE(mat[0][0] == Catch::Approx(0.f).margin(1e-6));
    REQUIRE(mat[0][2] == Catch::Approx(1.f));
    REQUIRE(mat[2][0] == Catch::Approx(-1.f));
    // Translation: 10 cm → 100 mm in Z
    REQUIRE(mat[0][3] == Catch::Approx(0.f).margin(1e-4));
    REQUIRE(mat[2][3] == Catch::Approx(100.f));

    // Apply to a single known point (0, 0, 1000 mm)
    dai::node::PointCloud::Impl impl;
    impl.setExtrinsics(mat);
    std::vector<dai::Point3f> pts = {{0.f, 0.f, 1000.f}};
    impl.applyTransformation(pts);

    // R_90y * (0,0,1000) + (0,0,100) = (1000, 0, 0) + (0, 0, 100) = (1000, 0, 100)
    REQUIRE(pts[0].x == Catch::Approx(1000.f));
    REQUIRE(pts[0].y == Catch::Approx(0.f));
    REQUIRE(pts[0].z == Catch::Approx(100.f));
}

// ============================================================================
// Camera-to-camera: inverse direction (CAM_C → CAM_B)
// ============================================================================
TEST_CASE("Camera-to-camera inverse direction", "[PointCloud][CalibrationHandler][CameraExtrinsics]") {
    auto handler = makeTwoCameraHandler(eye3(), {7.5f, 0.f, 0.f});

    auto fwd = handler.getCameraExtrinsics(dai::CameraBoardSocket::CAM_B, dai::CameraBoardSocket::CAM_C, false, dai::LengthUnit::MILLIMETER);
    auto inv = handler.getCameraExtrinsics(dai::CameraBoardSocket::CAM_C, dai::CameraBoardSocket::CAM_B, false, dai::LengthUnit::MILLIMETER);

    // Forward: +75 mm X, Inverse: -75 mm X
    REQUIRE(fwd[0][3] == Catch::Approx(75.f));
    REQUIRE(inv[0][3] == Catch::Approx(-75.f));

    // Applying forward then inverse should give back the original point
    dai::node::PointCloud::Impl implFwd, implInv;
    implFwd.setExtrinsics(fwd);
    implInv.setExtrinsics(inv);

    std::vector<dai::Point3f> pts = {{10.f, 20.f, 500.f}};
    auto original = pts;
    implFwd.applyTransformation(pts);
    implInv.applyTransformation(pts);

    REQUIRE(pts[0].x == Catch::Approx(original[0].x));
    REQUIRE(pts[0].y == Catch::Approx(original[0].y));
    REQUIRE(pts[0].z == Catch::Approx(original[0].z));
}

// ============================================================================
// Camera-to-camera: three-camera chain (CAM_A → CAM_B → CAM_C)
// ============================================================================
TEST_CASE("Three-camera chain extrinsics", "[PointCloud][CalibrationHandler][CameraExtrinsics]") {
    // CAM_A → CAM_B: identity, 5 cm X
    // CAM_B → CAM_C: identity, 3 cm X
    auto handler = makeThreeCameraHandler(eye3(), {5.f, 0.f, 0.f}, eye3(), {3.f, 0.f, 0.f});

    SECTION("CAM_A to CAM_C: translations add up") {
        auto mat = handler.getCameraExtrinsics(dai::CameraBoardSocket::CAM_A, dai::CameraBoardSocket::CAM_C, false, dai::LengthUnit::MILLIMETER);
        // 5 cm + 3 cm = 8 cm = 80 mm
        REQUIRE(mat[0][3] == Catch::Approx(80.f));
    }

    SECTION("CAM_A to CAM_B: only first link") {
        auto mat = handler.getCameraExtrinsics(dai::CameraBoardSocket::CAM_A, dai::CameraBoardSocket::CAM_B, false, dai::LengthUnit::MILLIMETER);
        // 5 cm = 50 mm
        REQUIRE(mat[0][3] == Catch::Approx(50.f));
    }

    SECTION("CAM_B to CAM_A: inverse of first link") {
        auto mat = handler.getCameraExtrinsics(dai::CameraBoardSocket::CAM_B, dai::CameraBoardSocket::CAM_A, false, dai::LengthUnit::MILLIMETER);
        REQUIRE(mat[0][3] == Catch::Approx(-50.f));
    }
}

// ============================================================================
// Camera-to-camera: spec translation vs measured translation
// ============================================================================
TEST_CASE("Spec translation vs measured translation", "[PointCloud][CalibrationHandler][CameraExtrinsics]") {
    // Measured: (7.5, 0.2, 0.1) cm   Spec: (7.5, 0, 0) cm
    auto handler = makeTwoCameraHandler(eye3(), {7.5f, 0.2f, 0.1f}, {7.5f, 0.f, 0.f});

    SECTION("useSpecTranslation=false uses measured translation") {
        auto mat = handler.getCameraExtrinsics(dai::CameraBoardSocket::CAM_B, dai::CameraBoardSocket::CAM_C, false, dai::LengthUnit::MILLIMETER);
        REQUIRE(mat[0][3] == Catch::Approx(75.f));
        REQUIRE(mat[1][3] == Catch::Approx(2.f));   // 0.2 cm → 2 mm
        REQUIRE(mat[2][3] == Catch::Approx(1.f));   // 0.1 cm → 1 mm
    }

    SECTION("useSpecTranslation=true uses spec translation") {
        auto mat = handler.getCameraExtrinsics(dai::CameraBoardSocket::CAM_B, dai::CameraBoardSocket::CAM_C, true, dai::LengthUnit::MILLIMETER);
        REQUIRE(mat[0][3] == Catch::Approx(75.f));
        REQUIRE(mat[1][3] == Catch::Approx(0.f));
        REQUIRE(mat[2][3] == Catch::Approx(0.f));
    }
}

// ============================================================================
// Camera-to-camera: rotation + translation combined transform on point cloud
// ============================================================================
TEST_CASE("Camera-to-camera 90-deg Z rotation applied to dense cloud", "[PointCloud][CalibrationHandler][CameraExtrinsics]") {
    // 90° Z rotation: (x,y,z) → (-y, x, z)
    auto handler = makeTwoCameraHandler(rot90z(), {5.f, 0.f, 0.f});  // 5 cm = 50 mm X

    auto mat =
        handler.getCameraExtrinsics(dai::CameraBoardSocket::CAM_B, dai::CameraBoardSocket::CAM_C, false, dai::LengthUnit::MILLIMETER);

    dai::node::PointCloud::Impl impl;
    constexpr unsigned W = 4, H = 4;
    constexpr float fx = 100.f, fy = 100.f, cx = 2.f, cy = 2.f;
    impl.setIntrinsics(fx, fy, cx, cy, W, H);
    impl.setExtrinsics(mat);

    auto depth = makeConstantDepth(W, H, 2000);  // 2 m
    auto pts = computeDense(impl, depth);
    auto before = pts;
    impl.applyTransformation(pts);

    for(size_t i = 0; i < pts.size(); ++i) {
        if(before[i].z > 0.f) {
            // 90° Z: x' = -y + 50, y' = x, z' = z
            REQUIRE(pts[i].x == Catch::Approx(-before[i].y + 50.f));
            REQUIRE(pts[i].y == Catch::Approx(before[i].x));
            REQUIRE(pts[i].z == Catch::Approx(before[i].z));
        }
    }
}

// ============================================================================
// Camera-to-camera: unit consistency between point cloud and extrinsics
// ============================================================================
TEST_CASE("Extrinsics unit must match point cloud unit", "[PointCloud][CalibrationHandler][CameraExtrinsics][LengthUnit]") {
    auto handler = makeTwoCameraHandler(eye3(), {10.f, 0.f, 0.f});  // 10 cm

    SECTION("Both in meters") {
        auto mat = handler.getCameraExtrinsics(dai::CameraBoardSocket::CAM_B, dai::CameraBoardSocket::CAM_C, false, dai::LengthUnit::METER);
        dai::node::PointCloud::Impl impl;
        impl.setLengthUnit(dai::LengthUnit::METER);
        impl.setIntrinsics(100.f, 100.f, 2.f, 2.f, 4, 4);
        impl.setExtrinsics(mat);

        auto depth = makeConstantDepth(4, 4, 1000);  // 1 m
        auto pts = computeDense(impl, depth);
        impl.applyTransformation(pts);

        // Centre pixel: x=0, y=0, z=1 m → after +0.1 m X shift
        auto& centre = pts[2 * 4 + 2];
        REQUIRE(centre.x == Catch::Approx(0.1f));
        REQUIRE(centre.z == Catch::Approx(1.f));
    }

    SECTION("Both in centimeters") {
        auto mat =
            handler.getCameraExtrinsics(dai::CameraBoardSocket::CAM_B, dai::CameraBoardSocket::CAM_C, false, dai::LengthUnit::CENTIMETER);
        dai::node::PointCloud::Impl impl;
        impl.setLengthUnit(dai::LengthUnit::CENTIMETER);
        impl.setIntrinsics(100.f, 100.f, 2.f, 2.f, 4, 4);
        impl.setExtrinsics(mat);

        auto depth = makeConstantDepth(4, 4, 1000);  // 1000 mm → 100 cm
        auto pts = computeDense(impl, depth);
        impl.applyTransformation(pts);

        auto& centre = pts[2 * 4 + 2];
        REQUIRE(centre.x == Catch::Approx(10.f));   // 10 cm
        REQUIRE(centre.z == Catch::Approx(100.f));   // 100 cm
    }
}

// ============================================================================
// Housing transform: identity housing → pure camera-to-origin
// ============================================================================
TEST_CASE("Housing identity transform equals camera-to-origin inverse", "[PointCloud][CalibrationHandler][Housing]") {
    // Housing origin = CAM_B (which IS the origin since toCameraSocket=AUTO).
    // Housing rotation = I, translation = (0,0,0).
    // So getHousingCalibration(CAM_B, AUTO, false, MM) should be identity.
    auto handler = makeHousingHandler(dai::CameraBoardSocket::CAM_B, eye3(), {0.f, 0.f, 0.f});

    auto mat = handler.getHousingCalibration(dai::CameraBoardSocket::CAM_B, dai::HousingCoordinateSystem::AUTO, false, dai::LengthUnit::MILLIMETER);

    // Should be identity
    for(int r = 0; r < 4; ++r)
        for(int c = 0; c < 4; ++c) REQUIRE(mat[r][c] == Catch::Approx(r == c ? 1.f : 0.f).margin(1e-5));
}

// ============================================================================
// Housing transform: pure translation offset
// ============================================================================
TEST_CASE("Housing pure translation offset", "[PointCloud][CalibrationHandler][Housing]") {
    // Housing at (5, 3, 0) cm from CAM_B (which is origin).
    // getHousingCalibration for CAM_B should give: cam → housing = inverse of housing-to-cam
    // housingToHousingOrigin = [I | (5,3,0)] cm  →  inv = [I | (-5,-3,0)]
    // camToOrigin = I (CAM_B is origin)
    // housingOriginToOrigin = I (housingOrigin=CAM_B is origin)
    // camToHousing = inv(housingToHousingOrigin) * inv(housingOriginToOrigin) * camToOrigin
    //             = [I|(-5,-3,0)] * I * I = [I|(-5,-3,0)] in cm → (-50,-30,0) mm
    auto handler = makeHousingHandler(dai::CameraBoardSocket::CAM_B, eye3(), {5.f, 3.f, 0.f});

    auto mat = handler.getHousingCalibration(dai::CameraBoardSocket::CAM_B, dai::HousingCoordinateSystem::AUTO, false, dai::LengthUnit::MILLIMETER);

    REQUIRE(mat[0][0] == Catch::Approx(1.f));
    REQUIRE(mat[0][3] == Catch::Approx(-50.f));
    REQUIRE(mat[1][3] == Catch::Approx(-30.f));
    REQUIRE(mat[2][3] == Catch::Approx(0.f).margin(1e-5));

    // Apply to point cloud
    dai::node::PointCloud::Impl impl;
    impl.setIntrinsics(100.f, 100.f, 2.f, 2.f, 4, 4);
    impl.setExtrinsics(mat);

    // Single point at origin of camera (0,0,1000mm)
    std::vector<dai::Point3f> pts = {{0.f, 0.f, 1000.f}};
    impl.applyTransformation(pts);

    REQUIRE(pts[0].x == Catch::Approx(-50.f));
    REQUIRE(pts[0].y == Catch::Approx(-30.f));
    REQUIRE(pts[0].z == Catch::Approx(1000.f));
}

// ============================================================================
// Housing transform: rotation (90° Z) without translation
// ============================================================================
TEST_CASE("Housing 90-deg Z rotation", "[PointCloud][CalibrationHandler][Housing]") {
    // Housing rotation = 90° Z, no translation, linked to CAM_B (origin).
    // housingToHousingOrigin = [R_90z | 0]
    // inv = [R_90z^T | 0]
    // R_90z^T = [[0,1,0],[-1,0,0],[0,0,1]]
    // Point (10, 0, 1000) in CAM_B → in housing: (0*10+1*0, -1*10+0*0, 1000) = (0, -10, 1000)
    auto handler = makeHousingHandler(dai::CameraBoardSocket::CAM_B, rot90z(), {0.f, 0.f, 0.f});

    auto mat = handler.getHousingCalibration(dai::CameraBoardSocket::CAM_B, dai::HousingCoordinateSystem::AUTO, false, dai::LengthUnit::MILLIMETER);

    dai::node::PointCloud::Impl impl;
    impl.setExtrinsics(mat);

    std::vector<dai::Point3f> pts = {{10.f, 0.f, 1000.f}};
    impl.applyTransformation(pts);

    REQUIRE(pts[0].x == Catch::Approx(0.f).margin(1e-4));
    REQUIRE(pts[0].y == Catch::Approx(-10.f));
    REQUIRE(pts[0].z == Catch::Approx(1000.f));
}

// ============================================================================
// Housing transform: rotation + translation combined
// ============================================================================
TEST_CASE("Housing rotation and translation combined", "[PointCloud][CalibrationHandler][Housing]") {
    // Housing at 90° Z rotation, translation (5, 0, 0) cm from CAM_B.
    // housingToHousingOrigin = [R_90z | (5,0,0)] cm
    // inv([R_90z|(5,0,0)]) = [R_90z^T | -R_90z^T * (5,0,0)]
    // R_90z^T = [[0,1,0],[-1,0,0],[0,0,1]]
    // -R_90z^T * (5,0,0) = -(0*5+1*0, -1*5+0*0, 0*5+0*0) = -(0, -5, 0) = (0, 5, 0) cm → (0, 50, 0) mm
    auto handler = makeHousingHandler(dai::CameraBoardSocket::CAM_B, rot90z(), {5.f, 0.f, 0.f});

    auto mat = handler.getHousingCalibration(dai::CameraBoardSocket::CAM_B, dai::HousingCoordinateSystem::AUTO, false, dai::LengthUnit::MILLIMETER);

    // Translation column
    REQUIRE(mat[0][3] == Catch::Approx(0.f).margin(1e-3));
    REQUIRE(mat[1][3] == Catch::Approx(50.f));
    REQUIRE(mat[2][3] == Catch::Approx(0.f).margin(1e-3));

    // Apply to point: (0, 0, 2000) → R_90z^T * (0,0,2000) + (0,50,0) = (0, 0, 2000) + (0, 50, 0) = (0, 50, 2000)
    dai::node::PointCloud::Impl impl;
    impl.setExtrinsics(mat);
    std::vector<dai::Point3f> pts = {{0.f, 0.f, 2000.f}};
    impl.applyTransformation(pts);

    REQUIRE(pts[0].x == Catch::Approx(0.f).margin(1e-3));
    REQUIRE(pts[0].y == Catch::Approx(50.f));
    REQUIRE(pts[0].z == Catch::Approx(2000.f));
}

// ============================================================================
// Housing transform: source camera different from housing origin
// ============================================================================
TEST_CASE("Housing transform with source camera not at housing origin", "[PointCloud][CalibrationHandler][Housing]") {
    // Setup: CAM_B → CAM_C (identity, 10 cm X), CAM_C is origin.
    // Housing linked to CAM_C (origin) with identity rotation, translation (2, 0, 0) cm.
    //
    // getHousingCalibration(CAM_B, AUTO, false, MM):
    //   housingToHousingOrigin = [I | (2,0,0)] cm, originSocket = CAM_C
    //   housingOriginToOrigin = getExtrinsicsToOrigin(CAM_C) = I  (CAM_C is origin)
    //   camToOrigin = getExtrinsicsToOrigin(CAM_B) = [I | (10,0,0)] cm  (CAM_B→CAM_C)
    //   inv(housingOriginToOrigin) = I   (origin→CAM_C)
    //   inv(housingToHousingOrigin) = [I | (-2,0,0)]  (housingOrigin→housing)
    //   camToHousingOrigin = I * [I|(10,0,0)] = [I|(10,0,0)]
    //   camToHousing = [I|(-2,0,0)] * [I|(10,0,0)] = [I|(8,0,0)] cm → 80mm
    dai::EepromData eeprom;
    eeprom.housingExtrinsics.rotationMatrix = eye3();
    eeprom.housingExtrinsics.translation = dai::Point3f(2.f, 0.f, 0.f);
    eeprom.housingExtrinsics.toCameraSocket = dai::CameraBoardSocket::CAM_C;

    dai::CalibrationHandler handler(eeprom);
    auto intr = defaultIntrinsics();
    handler.setCameraIntrinsics(dai::CameraBoardSocket::CAM_B, intr, 640, 480);
    handler.setCameraIntrinsics(dai::CameraBoardSocket::CAM_C, intr, 640, 480);
    handler.setCameraExtrinsics(dai::CameraBoardSocket::CAM_B, dai::CameraBoardSocket::CAM_C, eye3(), {10.f, 0.f, 0.f});

    auto mat = handler.getHousingCalibration(dai::CameraBoardSocket::CAM_B, dai::HousingCoordinateSystem::AUTO, false, dai::LengthUnit::MILLIMETER);

    REQUIRE(mat[0][3] == Catch::Approx(80.f));
    REQUIRE(mat[1][3] == Catch::Approx(0.f).margin(1e-4));
    REQUIRE(mat[2][3] == Catch::Approx(0.f).margin(1e-4));

    // Apply to dense cloud and verify shift
    dai::node::PointCloud::Impl impl;
    constexpr unsigned W = 4, H = 4;
    impl.setIntrinsics(100.f, 100.f, 2.f, 2.f, W, H);
    impl.setExtrinsics(mat);

    auto depth = makeConstantDepth(W, H, 500);
    auto pts = computeDense(impl, depth);
    auto before = pts;
    impl.applyTransformation(pts);

    for(size_t i = 0; i < pts.size(); ++i) {
        if(before[i].z > 0.f) {
            REQUIRE(pts[i].x == Catch::Approx(before[i].x + 80.f));
            REQUIRE(pts[i].y == Catch::Approx(before[i].y));
            REQUIRE(pts[i].z == Catch::Approx(before[i].z));
        }
    }
}

// ============================================================================
// Housing transform: spec translation (no database lookup)
// ============================================================================
TEST_CASE("Housing spec translation without database", "[PointCloud][CalibrationHandler][Housing]") {
    // Housing linked to CAM_B (origin) with spec translation (3, 1, 0) cm
    // (measured translation is (3.1, 1.2, 0.05) cm)
    auto handler = makeHousingHandler(dai::CameraBoardSocket::CAM_B, eye3(), {3.1f, 1.2f, 0.05f}, {3.f, 1.f, 0.f});

    auto matMeasured = handler.getHousingCalibration(dai::CameraBoardSocket::CAM_B, dai::HousingCoordinateSystem::AUTO, false, dai::LengthUnit::MILLIMETER);
    auto matSpec = handler.getHousingCalibration(dai::CameraBoardSocket::CAM_B, dai::HousingCoordinateSystem::AUTO, true, dai::LengthUnit::MILLIMETER);

    // Measured: translation = -(3.1, 1.2, 0.05) cm → (-31, -12, -0.5) mm
    REQUIRE(matMeasured[0][3] == Catch::Approx(-31.f));
    REQUIRE(matMeasured[1][3] == Catch::Approx(-12.f));
    REQUIRE(matMeasured[2][3] == Catch::Approx(-0.5f));

    // Spec: translation = -(3, 1, 0) cm → (-30, -10, 0) mm
    REQUIRE(matSpec[0][3] == Catch::Approx(-30.f));
    REQUIRE(matSpec[1][3] == Catch::Approx(-10.f));
    REQUIRE(matSpec[2][3] == Catch::Approx(0.f).margin(1e-4));
}

// ============================================================================
// Housing unit scaling: mm vs m
// ============================================================================
TEST_CASE("Housing transform unit scaling", "[PointCloud][CalibrationHandler][Housing][LengthUnit]") {
    auto handler = makeHousingHandler(dai::CameraBoardSocket::CAM_B, eye3(), {10.f, 5.f, 0.f});

    auto matMm = handler.getHousingCalibration(dai::CameraBoardSocket::CAM_B, dai::HousingCoordinateSystem::AUTO, false, dai::LengthUnit::MILLIMETER);
    auto matM = handler.getHousingCalibration(dai::CameraBoardSocket::CAM_B, dai::HousingCoordinateSystem::AUTO, false, dai::LengthUnit::METER);
    auto matCm = handler.getHousingCalibration(dai::CameraBoardSocket::CAM_B, dai::HousingCoordinateSystem::AUTO, false, dai::LengthUnit::CENTIMETER);

    // Housing (10, 5, 0) cm → inverse translation (-10, -5, 0) cm
    REQUIRE(matMm[0][3] == Catch::Approx(-100.f));  // -10 cm → -100 mm
    REQUIRE(matMm[1][3] == Catch::Approx(-50.f));

    REQUIRE(matM[0][3] == Catch::Approx(-0.1f));    // -10 cm → -0.1 m
    REQUIRE(matM[1][3] == Catch::Approx(-0.05f));

    REQUIRE(matCm[0][3] == Catch::Approx(-10.f));   // -10 cm
    REQUIRE(matCm[1][3] == Catch::Approx(-5.f));
}

// ============================================================================
// Housing + camera extrinsics: full pipeline depth → transform → verify
// ============================================================================
TEST_CASE("Housing full pipeline: depth to transformed cloud", "[PointCloud][CalibrationHandler][Housing][Integration]") {
    // CAM_B → CAM_C (identity, 5 cm X), housing at CAM_C with (2, 0, 0) cm translation
    dai::EepromData eeprom;
    eeprom.housingExtrinsics.rotationMatrix = eye3();
    eeprom.housingExtrinsics.translation = dai::Point3f(2.f, 0.f, 0.f);
    eeprom.housingExtrinsics.toCameraSocket = dai::CameraBoardSocket::CAM_C;

    dai::CalibrationHandler handler(eeprom);
    auto intr = defaultIntrinsics();
    handler.setCameraIntrinsics(dai::CameraBoardSocket::CAM_B, intr, 640, 480);
    handler.setCameraIntrinsics(dai::CameraBoardSocket::CAM_C, intr, 640, 480);
    handler.setCameraExtrinsics(dai::CameraBoardSocket::CAM_B, dai::CameraBoardSocket::CAM_C, eye3(), {5.f, 0.f, 0.f});

    auto mat = handler.getHousingCalibration(dai::CameraBoardSocket::CAM_B, dai::HousingCoordinateSystem::AUTO, false, dai::LengthUnit::METER);

    dai::node::PointCloud::Impl impl;
    constexpr unsigned W = 8, H = 8;
    impl.setIntrinsics(500.f, 500.f, 4.f, 4.f, W, H);
    impl.setLengthUnit(dai::LengthUnit::METER);
    impl.setExtrinsics(mat);

    auto depth = makeConstantDepth(W, H, 2000);  // 2 m in mm
    auto pts = computeDense(impl, depth);
    impl.applyTransformation(pts);
    auto sparse = impl.filterValidPoints(pts);

    // camToHousing translation = (5 - 2, 0, 0) = (3, 0, 0) cm = 0.03 m
    // Centre pixel (cx=4, cy=4): x=0.03, y=0, z=2.0
    auto& centre = pts[4 * W + 4];
    REQUIRE(centre.x == Catch::Approx(0.03f).epsilon(0.01));
    REQUIRE(centre.z == Catch::Approx(2.f));
}

// ============================================================================
// Camera extrinsics: missing camera throws
// ============================================================================
TEST_CASE("getCameraExtrinsics throws for missing camera", "[PointCloud][CalibrationHandler][CameraExtrinsics]") {
    auto handler = makeTwoCameraHandler(eye3(), {5.f, 0.f, 0.f});

    REQUIRE_THROWS(handler.getCameraExtrinsics(dai::CameraBoardSocket::CAM_A, dai::CameraBoardSocket::CAM_C, false, dai::LengthUnit::MILLIMETER));
}

// ============================================================================
// Housing: missing source camera throws
// ============================================================================
TEST_CASE("getHousingCalibration throws for missing source camera", "[PointCloud][CalibrationHandler][Housing]") {
    auto handler = makeHousingHandler(dai::CameraBoardSocket::CAM_B, eye3(), {0.f, 0.f, 0.f});

    REQUIRE_THROWS(handler.getHousingCalibration(dai::CameraBoardSocket::CAM_A, dai::HousingCoordinateSystem::AUTO, false, dai::LengthUnit::MILLIMETER));
}
