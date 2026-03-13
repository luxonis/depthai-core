#include <catch2/catch_all.hpp>
#include <cstdint>
#include <vector>

#include "depthai/common/Point3f.hpp"
#include "depthai/common/Point3fRGBA.hpp"
#include "depthai/pipeline/datatype/PointCloudConfig.hpp"
#include "depthai/pipeline/datatype/PointCloudData.hpp"
#include "depthai/properties/PointCloudProperties.hpp"
#include "depthai/utility/Serialization.hpp"

namespace {

/// Build a simple grid of Point3f for PointCloudData tests.
std::vector<dai::Point3f> makePointGrid(unsigned int w, unsigned int h, float z) {
    std::vector<dai::Point3f> pts;
    pts.reserve(w * h);
    for(unsigned int r = 0; r < h; ++r)
        for(unsigned int c = 0; c < w; ++c) pts.emplace_back(static_cast<float>(c), static_cast<float>(r), z);
    return pts;
}

}  // namespace

// ============================================================================
// PointCloudData – XYZ roundtrip through setPoints / getPoints
// ============================================================================
TEST_CASE("PointCloudData setPoints/getPoints roundtrip", "[PointCloudData]") {
    dai::PointCloudData pcd;
    auto src = makePointGrid(8, 8, 2.5f);
    pcd.setPoints(src);
    pcd.setWidth(8).setHeight(8);

    auto out = pcd.getPoints();
    REQUIRE(out.size() == src.size());
    for(size_t i = 0; i < src.size(); ++i) {
        REQUIRE(out[i].x == Catch::Approx(src[i].x));
        REQUIRE(out[i].y == Catch::Approx(src[i].y));
        REQUIRE(out[i].z == Catch::Approx(src[i].z));
    }
    REQUIRE_FALSE(pcd.isColor());
}

// ============================================================================
// PointCloudData – RGBA roundtrip
// ============================================================================
TEST_CASE("PointCloudData RGBA roundtrip", "[PointCloudData][Color]") {
    dai::PointCloudData pcd;
    std::vector<dai::Point3fRGBA> src;
    for(int i = 0; i < 12; ++i) src.emplace_back(float(i), float(i + 1), float(i + 2), uint8_t(i * 10), uint8_t(i * 20), uint8_t(i * 5), 255);
    pcd.setPointsRGB(src);
    pcd.setWidth(4).setHeight(3);  // 4×3 organized cloud matching 12 points
    REQUIRE(pcd.isColor());

    auto out = pcd.getPointsRGB();
    REQUIRE(out.size() == src.size());
    for(size_t i = 0; i < src.size(); ++i) {
        REQUIRE(out[i].x == Catch::Approx(src[i].x));
        REQUIRE(out[i].r == src[i].r);
        REQUIRE(out[i].a == 255);
    }
}

TEST_CASE("PointCloudData getPointsRGB throws when not color", "[PointCloudData][Color]") {
    dai::PointCloudData pcd;
    pcd.setPoints(makePointGrid(2, 2, 1.f));
    REQUIRE_THROWS_AS(pcd.getPointsRGB(), std::runtime_error);
}

// ============================================================================
// PointCloudData – getPoints extracts XYZ from RGBA data
// ============================================================================
TEST_CASE("PointCloudData getPoints on color data returns XYZ only", "[PointCloudData][Color]") {
    dai::PointCloudData pcd;
    std::vector<dai::Point3fRGBA> src = {{1.f, 2.f, 3.f, 10, 20, 30, 255}, {4.f, 5.f, 6.f, 40, 50, 60, 255}};
    pcd.setPointsRGB(src);
    pcd.setWidth(2).setHeight(1);

    auto xyz = pcd.getPoints();
    REQUIRE(xyz.size() == 2);
    REQUIRE(xyz[0].x == Catch::Approx(1.f));
    REQUIRE(xyz[1].z == Catch::Approx(6.f));
}

// ============================================================================
// PointCloudData – metadata setters and chaining
// ============================================================================
TEST_CASE("PointCloudData bounding box and metadata", "[PointCloudData][Metadata]") {
    dai::PointCloudData pcd;

    pcd.setMinX(-5.f).setMinY(-10.f).setMinZ(0.1f).setMaxX(5.f).setMaxY(10.f).setMaxZ(100.f).setInstanceNum(3);

    REQUIRE(pcd.getMinX() == Catch::Approx(-5.f));
    REQUIRE(pcd.getMaxZ() == Catch::Approx(100.f));
    REQUIRE(pcd.getInstanceNum() == 3);
}

// ============================================================================
// PointCloudData – updateBoundingBox
// ============================================================================
TEST_CASE("updateBoundingBox on XYZ points", "[PointCloudData][BoundingBox]") {
    dai::PointCloudData pcd;
    std::vector<dai::Point3f> pts = {{-3.f, 1.f, 2.f}, {5.f, -7.f, 10.f}, {0.f, 4.f, 0.5f}};
    pcd.setPoints(pts);
    pcd.setWidth(3).setHeight(1);
    pcd.updateBoundingBox();

    REQUIRE(pcd.getMinX() == Catch::Approx(-3.f));
    REQUIRE(pcd.getMaxX() == Catch::Approx(5.f));
    REQUIRE(pcd.getMinY() == Catch::Approx(-7.f));
    REQUIRE(pcd.getMaxY() == Catch::Approx(4.f));
    REQUIRE(pcd.getMinZ() == Catch::Approx(0.5f));
    REQUIRE(pcd.getMaxZ() == Catch::Approx(10.f));
}

TEST_CASE("updateBoundingBox skips invalid z<=0 points", "[PointCloudData][BoundingBox]") {
    dai::PointCloudData pcd;
    // Two invalid (z<=0), one valid
    std::vector<dai::Point3f> pts = {{-100.f, -200.f, 0.f}, {999.f, 999.f, -1.f}, {2.f, 3.f, 4.f}};
    pcd.setPoints(pts);
    pcd.setWidth(3).setHeight(1);
    pcd.updateBoundingBox();

    // Only the z=4 point matters
    REQUIRE(pcd.getMinX() == Catch::Approx(2.f));
    REQUIRE(pcd.getMaxX() == Catch::Approx(2.f));
    REQUIRE(pcd.getMinY() == Catch::Approx(3.f));
    REQUIRE(pcd.getMaxY() == Catch::Approx(3.f));
    REQUIRE(pcd.getMinZ() == Catch::Approx(4.f));
    REQUIRE(pcd.getMaxZ() == Catch::Approx(4.f));
}

TEST_CASE("updateBoundingBox all invalid yields zeros", "[PointCloudData][BoundingBox]") {
    dai::PointCloudData pcd;
    std::vector<dai::Point3f> pts = {{1.f, 2.f, 0.f}, {-1.f, -2.f, -5.f}};
    pcd.setPoints(pts);
    pcd.setWidth(2).setHeight(1);
    pcd.updateBoundingBox();

    REQUIRE(pcd.getMinX() == 0.f);
    REQUIRE(pcd.getMaxX() == 0.f);
    REQUIRE(pcd.getMinZ() == 0.f);
    REQUIRE(pcd.getMaxZ() == 0.f);
}

TEST_CASE("updateBoundingBox empty cloud yields zeros", "[PointCloudData][BoundingBox]") {
    dai::PointCloudData pcd;
    pcd.setPoints({});
    pcd.setWidth(0).setHeight(1);
    pcd.updateBoundingBox();

    REQUIRE(pcd.getMinX() == 0.f);
    REQUIRE(pcd.getMaxZ() == 0.f);
}

TEST_CASE("updateBoundingBox on RGBA points", "[PointCloudData][BoundingBox][Color]") {
    dai::PointCloudData pcd;
    std::vector<dai::Point3fRGBA> pts = {
        {1.f, 2.f, 3.f, 0, 0, 0, 255},
        {-4.f, 5.f, 0.1f, 0, 0, 0, 255},
        {10.f, -8.f, 20.f, 0, 0, 0, 255},
        {0.f, 0.f, 0.f, 0, 0, 0, 255},  // invalid, z=0
    };
    pcd.setPointsRGB(pts);
    pcd.setWidth(4).setHeight(1);
    pcd.updateBoundingBox();

    REQUIRE(pcd.getMinX() == Catch::Approx(-4.f));
    REQUIRE(pcd.getMaxX() == Catch::Approx(10.f));
    REQUIRE(pcd.getMinY() == Catch::Approx(-8.f));
    REQUIRE(pcd.getMaxY() == Catch::Approx(5.f));
    REQUIRE(pcd.getMinZ() == Catch::Approx(0.1f));
    REQUIRE(pcd.getMaxZ() == Catch::Approx(20.f));
}

TEST_CASE("updateBoundingBox returns *this for chaining", "[PointCloudData][BoundingBox]") {
    dai::PointCloudData pcd;
    pcd.setPoints({{1.f, 2.f, 3.f}});
    pcd.setWidth(1).setHeight(1);

    // Should be chainable
    auto& ref = pcd.updateBoundingBox();
    REQUIRE(&ref == &pcd);
    REQUIRE(pcd.getMinZ() == Catch::Approx(3.f));
}

// ============================================================================
// PointCloudData – serialization roundtrip
// ============================================================================
TEST_CASE("PointCloudData metadata serialization roundtrip", "[PointCloudData][Serialization]") {
    dai::PointCloudData pcd;
    pcd.setWidth(640).setHeight(480).setInstanceNum(7);
    pcd.setMinX(-100.f).setMaxX(100.f).setMinY(-50.f).setMaxY(50.f).setMinZ(0.5f).setMaxZ(20.f);
    pcd.setSequenceNum(42);

    std::vector<uint8_t> meta;
    dai::DatatypeEnum dt;
    pcd.serialize(meta, dt);
    REQUIRE(dt == dai::DatatypeEnum::PointCloudData);
    REQUIRE_FALSE(meta.empty());

    dai::PointCloudData pcd2;
    dai::utility::deserialize(meta, pcd2);

    REQUIRE(pcd2.getWidth() == 640);
    REQUIRE(pcd2.getHeight() == 480);
    REQUIRE(pcd2.getInstanceNum() == 7);
    REQUIRE(pcd2.getMinX() == Catch::Approx(-100.f));
    REQUIRE(pcd2.getMaxZ() == Catch::Approx(20.f));
    REQUIRE(pcd2.getSequenceNum() == 42);
}

// ============================================================================
// PointCloudConfig – defaults and setters
// ============================================================================
TEST_CASE("PointCloudConfig defaults and setters", "[PointCloudConfig]") {
    dai::PointCloudConfig cfg;

    // Default is not organized (filter invalid points)
    REQUIRE_FALSE(cfg.getOrganized());

    // Default is identity matrix
    auto m = cfg.getTransformationMatrix();
    for(int i = 0; i < 4; ++i)
        for(int j = 0; j < 4; ++j) REQUIRE(m[i][j] == Catch::Approx(i == j ? 1.f : 0.f));

    cfg.setOrganized(true);
    REQUIRE(cfg.getOrganized());

    std::array<std::array<float, 4>, 4> mat = {{{0, -1, 0, 1}, {1, 0, 0, 2}, {0, 0, 1, 3}, {0, 0, 0, 1}}};
    cfg.setTransformationMatrix(mat);
    auto out = cfg.getTransformationMatrix();
    for(int i = 0; i < 4; ++i)
        for(int j = 0; j < 4; ++j) REQUIRE(out[i][j] == Catch::Approx(mat[i][j]));
}

// ============================================================================
// PointCloudConfig – serialization roundtrip
// ============================================================================
TEST_CASE("PointCloudConfig serialization roundtrip", "[PointCloudConfig][Serialization]") {
    dai::PointCloudConfig cfg;
    cfg.setOrganized(true);
    std::array<std::array<float, 4>, 4> mat = {{{0.5f, 0, 0, 1}, {0, 0.5f, 0, 2}, {0, 0, 0.5f, 3}, {0, 0, 0, 1}}};
    cfg.setTransformationMatrix(mat);

    std::vector<uint8_t> meta;
    dai::DatatypeEnum dt;
    cfg.serialize(meta, dt);
    REQUIRE(dt == dai::DatatypeEnum::PointCloudConfig);
    REQUIRE_FALSE(meta.empty());

    dai::PointCloudConfig cfg2;
    dai::utility::deserialize(meta, cfg2);
    REQUIRE(cfg2.getOrganized());
    auto out = cfg2.getTransformationMatrix();
    for(int i = 0; i < 4; ++i)
        for(int j = 0; j < 4; ++j) REQUIRE(out[i][j] == Catch::Approx(mat[i][j]));
}

// ============================================================================
// PointCloudProperties – defaults and serialization roundtrip
// ============================================================================
TEST_CASE("PointCloudProperties defaults and serialization", "[PointCloudProperties]") {
    dai::PointCloudProperties props;
    REQUIRE(props.numFramesPool == 4);
    REQUIRE_FALSE(props.initialConfig.getOrganized());  // default organized=false

    props.numFramesPool = 16;
    props.initialConfig.setOrganized(true);

    auto ser = dai::utility::serialize(props);
    dai::PointCloudProperties props2;
    dai::utility::deserialize(ser, props2);
    REQUIRE(props2.numFramesPool == 16);
    REQUIRE(props2.initialConfig.getOrganized());
}

// ============================================================================
// Point3f / Point3fRGBA compile-time sizes
// ============================================================================
TEST_CASE("Point3f and Point3fRGBA sizes", "[Point3f]") {
    static_assert(sizeof(dai::Point3f) == 12, "Point3f must be 12 bytes");
    static_assert(sizeof(dai::Point3fRGBA) == 16, "Point3fRGBA must be 16 bytes");

    dai::Point3f p(1.f, 2.f, 3.f);
    REQUIRE(p.x == 1.f);

    dai::Point3fRGBA pc(1.f, 2.f, 3.f, 10, 20, 30, 200);
    REQUIRE(pc.a == 200);
}
