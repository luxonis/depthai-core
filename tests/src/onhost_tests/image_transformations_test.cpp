// Add test case for every transformation + edge cases

#include "depthai/common/ImgTransformations.hpp"
#include "depthai/utility/ImageManipImpl.hpp"
#define CATCH_CONFIG_MAIN

#include <catch2/catch_all.hpp>
#include <chrono>
#include <iostream>
#include <thread>

#include "depthai/depthai.hpp"

bool approxIdentity(const std::array<std::array<float, 3>, 3>& mat, float eps = 1e-4f) {
    for(int i = 0; i < 3; i++) {
        for(int j = 0; j < 3; j++) {
            float expected = (i == j) ? 1.0f : 0.0f;
            if(std::abs(mat[i][j] - expected) > eps) return false;
        }
    }
    return true;
}

inline std::array<std::array<float, 3>, 3> matmul(std::array<std::array<float, 3>, 3> A, std::array<std::array<float, 3>, 3> B) {
    return {{{A[0][0] * B[0][0] + A[0][1] * B[1][0] + A[0][2] * B[2][0],
              A[0][0] * B[0][1] + A[0][1] * B[1][1] + A[0][2] * B[2][1],
              A[0][0] * B[0][2] + A[0][1] * B[1][2] + A[0][2] * B[2][2]},
             {A[1][0] * B[0][0] + A[1][1] * B[1][0] + A[1][2] * B[2][0],
              A[1][0] * B[0][1] + A[1][1] * B[1][1] + A[1][2] * B[2][1],
              A[1][0] * B[0][2] + A[1][1] * B[1][2] + A[1][2] * B[2][2]},
             {A[2][0] * B[0][0] + A[2][1] * B[1][0] + A[2][2] * B[2][0],
              A[2][0] * B[0][1] + A[2][1] * B[1][1] + A[2][2] * B[2][1],
              A[2][0] * B[0][2] + A[2][1] * B[1][2] + A[2][2] * B[2][2]}}};
}

inline std::array<float, 2> matvecmul(std::array<std::array<float, 3>, 3> M, std::array<float, 2> vec) {
    auto x = M[0][0] * vec[0] + M[0][1] * vec[1] + M[0][2];
    auto y = M[1][0] * vec[0] + M[1][1] * vec[1] + M[1][2];
    auto z = M[2][0] * vec[0] + M[2][1] * vec[1] + M[2][2];
    return {x / z, y / z};
}

int testPadding() {
    auto sourceImageFrame = std::make_shared<dai::ImgFrame>();
    sourceImageFrame->setWidth(1920);
    sourceImageFrame->setHeight(1080);
    sourceImageFrame->setSourceSize(1920, 1080);
    sourceImageFrame->transformation = dai::ImgTransformation(1920, 1080);
    sourceImageFrame->transformation.addPadding(100, 200, 300, 400);
    sourceImageFrame->setWidth(2620);
    sourceImageFrame->setHeight(1380);
    REQUIRE(sourceImageFrame->validateTransformations());

    dai::Rect sourceRect{0.3, 0.7, 0.1, 0.1};
    auto outRect = sourceImageFrame->remapRectToSource(sourceRect);
    REQUIRE_THAT(0.25, Catch::Matchers::WithinAbs(outRect.x, 0.01));
    REQUIRE_THAT(0.80, Catch::Matchers::WithinAbs(outRect.y, 0.01));
    REQUIRE_THAT(0.13, Catch::Matchers::WithinAbs(outRect.width, 0.01));
    REQUIRE_THAT(0.12, Catch::Matchers::WithinAbs(outRect.height, 0.01));
    auto reverseBackRect = sourceImageFrame->remapRectFromSource(outRect);
    REQUIRE_THAT(reverseBackRect.x, Catch::Matchers::WithinAbs(sourceRect.x, 0.01));
    REQUIRE_THAT(reverseBackRect.y, Catch::Matchers::WithinAbs(sourceRect.y, 0.01));
    REQUIRE_THAT(reverseBackRect.width, Catch::Matchers::WithinAbs(sourceRect.width, 0.01));
    REQUIRE_THAT(reverseBackRect.height, Catch::Matchers::WithinAbs(sourceRect.height, 0.01));
    return 0;
}

int testCropping() {
    auto sourceImageFrame = std::make_shared<dai::ImgFrame>();
    sourceImageFrame->setWidth(1920);
    sourceImageFrame->setHeight(1080);
    sourceImageFrame->setSourceSize(1920, 1080);
    float x = 100;
    float y = 200;
    float width = 300;
    float height = 400;

    sourceImageFrame->transformation = dai::ImgTransformation(1920, 1080, width, height);
    sourceImageFrame->transformation.addCrop(x, y, width, height);
    sourceImageFrame->setSize(width, height);
    REQUIRE(sourceImageFrame->validateTransformations());

    dai::Rect sourceRect{150, 250, 50, 50};
    auto outRect = sourceImageFrame->remapRectToSource(sourceRect);
    REQUIRE_THAT(250, Catch::Matchers::WithinAbs(outRect.x, 0.01));
    REQUIRE_THAT(450, Catch::Matchers::WithinAbs(outRect.y, 0.01));
    REQUIRE_THAT(50, Catch::Matchers::WithinAbs(outRect.width, 0.01));
    REQUIRE_THAT(50, Catch::Matchers::WithinAbs(outRect.height, 0.01));
    auto reverseBackRect = sourceImageFrame->remapRectFromSource(outRect);
    REQUIRE_THAT(reverseBackRect.x, Catch::Matchers::WithinAbs(sourceRect.x, 0.01));
    REQUIRE_THAT(reverseBackRect.y, Catch::Matchers::WithinAbs(sourceRect.y, 0.01));
    REQUIRE_THAT(reverseBackRect.width, Catch::Matchers::WithinAbs(sourceRect.width, 0.01));
    REQUIRE_THAT(reverseBackRect.height, Catch::Matchers::WithinAbs(sourceRect.height, 0.01));
    return 0;
}

int testFlipping() {
    auto sourceImageFrame = std::make_shared<dai::ImgFrame>();
    sourceImageFrame->setWidth(1920);
    sourceImageFrame->setHeight(1080);
    sourceImageFrame->setSourceSize(1920, 1080);
    sourceImageFrame->transformation = dai::ImgTransformation(1920, 1080);
    sourceImageFrame->transformation.addFlipHorizontal();
    REQUIRE(sourceImageFrame->validateTransformations());
    dai::Rect sourceRect{100, 0, 300, 300};
    auto outRect = sourceImageFrame->remapRectToSource(sourceRect);
    REQUIRE_THAT(1520, Catch::Matchers::WithinAbs(outRect.x, 0.01));
    REQUIRE_THAT(0, Catch::Matchers::WithinAbs(outRect.y, 0.01));
    REQUIRE_THAT(300, Catch::Matchers::WithinAbs(outRect.width, 0.01));
    REQUIRE_THAT(300, Catch::Matchers::WithinAbs(outRect.height, 0.01));
    auto reverseBackRect = sourceImageFrame->remapRectFromSource(outRect);
    REQUIRE_THAT(reverseBackRect.x, Catch::Matchers::WithinAbs(sourceRect.x, 0.01));
    REQUIRE_THAT(reverseBackRect.y, Catch::Matchers::WithinAbs(sourceRect.y, 0.01));
    REQUIRE_THAT(reverseBackRect.width, Catch::Matchers::WithinAbs(sourceRect.width, 0.01));
    REQUIRE_THAT(reverseBackRect.height, Catch::Matchers::WithinAbs(sourceRect.height, 0.01));
    return 0;
}

int testScale() {
    auto sourceImageFrame = std::make_shared<dai::ImgFrame>();
    sourceImageFrame->setWidth(1920);
    sourceImageFrame->setHeight(1080);
    sourceImageFrame->setSourceSize(1920, 1080);

    sourceImageFrame->transformation = dai::ImgTransformation(1920, 1080);
    sourceImageFrame->transformation.addScale(12, 0.3);
    sourceImageFrame->setWidth(std::round(12 * 1920));
    sourceImageFrame->setHeight(std::round(0.3 * 1080));
    REQUIRE(sourceImageFrame->validateTransformations());

    dai::Rect sourceRect{0.3, 0.7, 0.1, 0.1};
    auto outRect = sourceImageFrame->remapRectToSource(sourceRect);
    REQUIRE_THAT(0.3, Catch::Matchers::WithinAbs(outRect.x, 0.01));
    REQUIRE_THAT(0.7, Catch::Matchers::WithinAbs(outRect.y, 0.01));
    REQUIRE_THAT(0.1, Catch::Matchers::WithinAbs(outRect.width, 0.01));
    REQUIRE_THAT(0.1, Catch::Matchers::WithinAbs(outRect.height, 0.01));
    auto reverseBackRect = sourceImageFrame->remapRectFromSource(outRect);
    REQUIRE_THAT(reverseBackRect.x, Catch::Matchers::WithinAbs(sourceRect.x, 0.01));
    REQUIRE_THAT(reverseBackRect.y, Catch::Matchers::WithinAbs(sourceRect.y, 0.01));
    REQUIRE_THAT(reverseBackRect.width, Catch::Matchers::WithinAbs(sourceRect.width, 0.01));
    REQUIRE_THAT(reverseBackRect.height, Catch::Matchers::WithinAbs(sourceRect.height, 0.01));
    return 0;
}

int testRotation(dai::Point2f inPoint,
                 float angle,
                 dai::Point2f outPointCheck,
                 dai::Point2f rotationPoint = dai::Point2f(960, 540),
                 dai::Point2f backPointCheck = dai::Point2f(0, 0)) {
    auto sourceImageFrame = std::make_shared<dai::ImgFrame>();
    sourceImageFrame->setWidth(1920);
    sourceImageFrame->setHeight(1080);
    sourceImageFrame->setSourceSize(1920, 1080);
    sourceImageFrame->transformation = dai::ImgTransformation(1920, 1080);
    sourceImageFrame->transformation.addRotation(angle, rotationPoint);
    REQUIRE(sourceImageFrame->validateTransformations());

    auto outPoint = sourceImageFrame->remapPointToSource(inPoint);
    REQUIRE_THAT(outPointCheck.x, Catch::Matchers::WithinAbs(outPoint.x, 0.01));
    REQUIRE_THAT(outPointCheck.y, Catch::Matchers::WithinAbs(outPoint.y, 0.01));
    auto reverseBackPoint = sourceImageFrame->remapPointFromSource(outPoint);
    if(backPointCheck.x != 0 || backPointCheck.y != 0) {
        inPoint = backPointCheck;
    }
    REQUIRE_THAT(reverseBackPoint.x, Catch::Matchers::WithinAbs(inPoint.x, 0.01));
    REQUIRE_THAT(reverseBackPoint.y, Catch::Matchers::WithinAbs(inPoint.y, 0.01));
    return 0;
}

TEST_CASE("testCropping") {
    testCropping();
}

TEST_CASE("testPadding") {
    testPadding();
}

TEST_CASE("testScale") {
    testScale();
}

TEST_CASE("testFlip") {
    testFlipping();
}

// -----------------------------------------------------------------------------
// imageSizeConsistency
// Purpose:
//   Ensures that ImgFrame correctly stores and reports its width/height and
//   source width/height. Changing the frame size must update getters properly.
//   Verifies basic integrity of ImgFrame dimension handling.
// -----------------------------------------------------------------------------
TEST_CASE("imageSizeConsistency") {
    dai::ImgFrame f;
    f.setSize(1920, 1080);
    f.setSourceSize(1920, 1080);

    REQUIRE(f.getWidth() == 1920);
    REQUIRE(f.getHeight() == 1080);
    REQUIRE(f.getSourceWidth() == 1920);
    REQUIRE(f.getSourceHeight() == 1080);

    f.setSize(1280, 720);
    REQUIRE(f.getWidth() == 1280);
    REQUIRE(f.getHeight() == 720);
}

// -----------------------------------------------------------------------------
// identityTransformation
// Purpose:
//   Verifies that a default ImgTransformation (identity mapping) does not alter
//   rectangles when remapped to/from source space. A round-trip remap of a Rect
//   should return exactly the same Rect (within floating-point tolerance).
// -----------------------------------------------------------------------------
TEST_CASE("identityTransformation") {
    dai::ImgFrame f;
    f.setSize(1920, 1080);
    f.setSourceSize(1920, 1080);
    f.transformation = dai::ImgTransformation(1920, 1080);

    dai::Rect r{100, 200, 300, 400};

    auto toSrc = f.remapRectToSource(r);
    auto back = f.remapRectFromSource(toSrc);

    REQUIRE_THAT(back.x, Catch::Matchers::WithinAbs(r.x, 1e-6));
    REQUIRE_THAT(back.y, Catch::Matchers::WithinAbs(r.y, 1e-6));
    REQUIRE_THAT(back.width, Catch::Matchers::WithinAbs(r.width, 1e-6));
    REQUIRE_THAT(back.height, Catch::Matchers::WithinAbs(r.height, 1e-6));
}

// -----------------------------------------------------------------------------
// invalidTransformations
// Purpose:
//   Confirms that validateTransformations() correctly flags invalid operations.
//   Here, adding a crop region completely outside the image must invalidate the
//   transformation chain. Ensures robust error detection for misconfigured ops.
// -----------------------------------------------------------------------------
TEST_CASE("invalidTransformations") {
    dai::ImgFrame f;
    f.setSize(1920, 1080);
    f.setSourceSize(1920, 1080);

    f.transformation = dai::ImgTransformation(1920, 1080);
    f.transformation.addCrop(2000, 2000, 100, 100);  // outside frame
    REQUIRE(!f.validateTransformations());
}

// -----------------------------------------------------------------------------
// strideAndPlaneHeight
// Purpose:
//   Ensures that ImgFrame computes plane height and stride correctly for
//   YUV420p images. Plane height must match image height, and stride must be
//   ≥ width to satisfy alignment/padding requirements.
// -----------------------------------------------------------------------------
TEST_CASE("strideAndPlaneHeight") {
    dai::ImgFrame f;
    f.setSize(1920, 1080);
    f.setType(dai::ImgFrame::Type::YUV420p);

    REQUIRE(f.getPlaneHeight() == 1080);
    REQUIRE(f.getStride() >= 1920);  // Y plane
}

// -----------------------------------------------------------------------------
// cloneFrame
// Purpose:
//   Tests deep copy behavior of ImgFrame:
//     • Transformation and size information must be preserved in the clone.
//     • Both original and clone must remain independently valid.
//     • Changing the original must NOT affect the clone (deep copy).
//   Also verifies round-trip mapping consistency after scaling.
// -----------------------------------------------------------------------------
TEST_CASE("cloneFrame") {
    dai::ImgFrame f1;
    f1.setSourceSize(1920, 1080);
    f1.setSize(1920, 1080);

    f1.transformation = dai::ImgTransformation(1920, 1080);
    f1.transformation.addScale(1.25f, 0.75f);

    // YOU MUST UPDATE OUTPUT SIZE!
    unsigned newW = std::round(1920 * 1.25f);
    unsigned newH = std::round(1080 * 0.75f);
    f1.setSize(newW, newH);

    REQUIRE(f1.validateTransformations());

    auto f2 = f1.clone();
    REQUIRE(f2->validateTransformations());
    REQUIRE(f2->getWidth() == f1.getWidth());

    REQUIRE(f2->remapPointToSource({10, 10}).x == f1.remapPointToSource({10, 10}).x);

    // Deep copy check:
    f1.setSize(100, 100);
    REQUIRE(f2->getWidth() != 100);
}

// -----------------------------------------------------------------------------
// testRotation
// Purpose:
//   Validates rotation behavior in ImgTransformation:
//     1. transformPoint() followed by invTransformPoint() must return the
//        original point (numerical invertibility).
//     2. The helper testRotation() is used to verify known rotation results
//        for various points and angles, ensuring rotation math correctness.
//
//   Confirms stable handling of arbitrary rotation centers, angles, and edge
//   cases (e.g., 0°, ±90°, off-center rotations).
// -----------------------------------------------------------------------------
TEST_CASE("testRotation") {
    auto transformation = dai::ImgTransformation(1920, 1080);
    transformation.addRotation(49, dai::Point2f(102, 303));
    auto pt = dai::Point2f(56, 44);
    auto ptTr = transformation.transformPoint(pt);
    auto ptBack = transformation.invTransformPoint(ptTr);
    REQUIRE_THAT(ptBack.x, Catch::Matchers::WithinAbs(pt.x, 0.01));
    REQUIRE_THAT(ptBack.y, Catch::Matchers::WithinAbs(pt.y, 0.01));

    testRotation(dai::Point2f(960, 540), 0, dai::Point2f(960, 540));
    testRotation(dai::Point2f(0, 0), 0, dai::Point2f(0, 0));
    testRotation(dai::Point2f(1000, 1000), 0, dai::Point2f(1000, 1000));
    testRotation(dai::Point2f(420, 1080), -90, dai::Point2f(420, 0));
    testRotation(dai::Point2f(420, 1080), 90, dai::Point2f(1500, 1080));
    testRotation(dai::Point2f(1920, 0), 90, dai::Point2f(420, -420));
    testRotation(dai::Point2f(0, 0), 90, dai::Point2f(0, 0), dai::Point2f(0, 0), dai::Point2f(0, 0));
}

// -----------------------------------------------------------------------------
// interFrameRemap
// Purpose:
//   Validates full cross-frame geometric remapping between two ImgFrames that
//   each have different intrinsic matrices, different output resolutions, and
//   different transformations (including rotations).
//
//   This test checks two key properties:
//
//   1. Forward mapping correctness:
//        - A point in frame1 is remapped to frame2.
//        - The result must match the manually constructed reference transform:
//            mat2 * intr2 * intr1inv * mat1inv
//        (This is the full chain: frame1 → sensor → normalized → sensor2 → frame2.)
//
//   2. Round-trip invertibility:
//        - Mapping point: frame1 → frame2 → frame1
//        - The result must return to the original point (within tolerance).
//
//   This test ensures:
//      • Intrinsic/inverse intrinsic usage is correct.
//      • Matrix composition (M2 * K2 * K1⁻¹ * M1⁻¹) matches library behavior.
//      • remapPointBetweenFrames() produces accurate, invertible mappings.
//      • Transformations involving rotations and differing resolutions remain
//        mathematically consistent.
// -----------------------------------------------------------------------------
TEST_CASE("interFrameRemap") {
    std::array<std::array<float, 3>, 3> intr1 = {{{784.8082885742188, 0.0, 652.2084350585938}, {0.0, 786.7345581054688, 406.1820373535156}, {0.0, 0.0, 1.0}}};
    auto intr1inv = dai::impl::getInverse(intr1);
    std::array<std::array<float, 3>, 3> intr2 = {{{3105.2021484375, 0.0, 1877.4822998046875}, {0.0, 3113.031494140625, 1128.080078125}, {0.0, 0.0, 1.0}}};
    auto intr2inv = dai::impl::getInverse(intr2);
    auto tr1 = dai::ImgTransformation(1920, 1080, intr1);
    auto tr2 = dai::ImgTransformation(1280, 720, intr2);

    tr1.addRotation(90, dai::Point2f(960, 540));
    tr2.addRotation(30, dai::Point2f(300, 400));

    dai::ImgFrame frame1;
    frame1.setSize(1920, 1080);
    frame1.setSourceSize(1920, 1080);
    frame1.setInstanceNum(1);
    frame1.transformation = tr1;

    dai::ImgFrame frame2;
    frame2.setSize(1280, 720);
    frame2.setSourceSize(1280, 720);
    frame1.setInstanceNum(2);
    frame2.transformation = tr2;

    auto mat1inv = tr1.getMatrixInv();
    auto mat2 = tr2.getMatrix();

    auto mat3 = matmul(mat2, matmul(intr2, matmul(intr1inv, mat1inv)));

    dai::Point2f pt(222, 333);
    auto pt1 = frame1.remapPointBetweenFrames(pt, frame1, frame2);
    auto pt2 = frame2.remapPointBetweenFrames(pt1, frame2, frame1);

    auto ptref = matvecmul(mat3, {pt.x, pt.y});

    REQUIRE_THAT(ptref[0], Catch::Matchers::WithinAbs(pt1.x, 0.01));
    REQUIRE_THAT(ptref[1], Catch::Matchers::WithinAbs(pt1.y, 0.01));

    REQUIRE_THAT(pt.x, Catch::Matchers::WithinAbs(pt2.x, 0.01));
}

// -----------------------------------------------------------------------------
// transformationCompositionOrder
// Purpose:
//   Verifies correct ordering and combination of ImgTransformation operations.
//   A sequence of crop → flip → padding → rotation → scale must still allow
//   round-trip point mapping (point → source → point). Confirms transform
//   composition math is correct and stable across mixed operations.
// -----------------------------------------------------------------------------
TEST_CASE("transformationCompositionOrder") {
    dai::ImgFrame f;
    f.setSize(1920, 1080);
    f.setSourceSize(1920, 1080);
    f.transformation = dai::ImgTransformation(1920, 1080);

    // Apply in this order:
    f.transformation.addCrop(100, 200, 1000, 500);
    f.transformation.addFlipHorizontal();
    f.transformation.addPadding(10, 20, 30, 40);
    f.transformation.addRotation(17, dai::Point2f(600, 300));
    f.transformation.addScale(1.25f, 0.75f);

    // 🔹 Sync ImgFrame size with transformation size
    {
        auto [tw, th] = f.transformation.getSize();
        f.setSize(static_cast<unsigned>(tw), static_cast<unsigned>(th));
    }

    REQUIRE(f.validateTransformations());

    dai::Point2f p{300, 400};
    auto p1 = f.remapPointToSource(p);
    auto p2 = f.remapPointFromSource(p1);

    REQUIRE_THAT(p2.x, Catch::Matchers::WithinAbs(p.x, 0.01f));
    REQUIRE_THAT(p2.y, Catch::Matchers::WithinAbs(p.y, 0.01f));
}

// -----------------------------------------------------------------------------
// doubleRotation
// Purpose:
//   Ensures numerical stability and correctness when transformations undo
//   each other. Rotating by +30° then -30° about the same center should yield
//   an identity effect. Round-trip point mapping should return the input point
//   (within tolerance). Confirms rotation-inverse math works properly.
// -----------------------------------------------------------------------------
TEST_CASE("doubleRotation") {
    dai::ImgFrame f;
    f.setSourceSize(1920, 1080);
    f.setSize(1920, 1080);
    f.transformation = dai::ImgTransformation(1920, 1080);

    f.transformation.addRotation(30, {960, 540});
    f.transformation.addRotation(-30, {960, 540});

    REQUIRE(f.validateTransformations());

    dai::Point2f p{100, 200};
    auto back = f.remapPointFromSource(f.remapPointToSource(p));

    REQUIRE_THAT(back.x, Catch::Matchers::WithinAbs(p.x, 0.1));
    REQUIRE_THAT(back.y, Catch::Matchers::WithinAbs(p.y, 0.1));
}

// -----------------------------------------------------------------------------
// flipRotateFlip
// Purpose:
//   Tests complex symmetry relationships. A flip → rotation → flip sequence
//   should behave as a rotation around a mirrored frame. Round-trip point
//   mapping must still return original point. This verifies correct handling
//   of transforms with sign inversions (flips) and rotation combined.
// -----------------------------------------------------------------------------
TEST_CASE("flipRotateFlip") {
    dai::ImgFrame f;
    f.setSourceSize(1920, 1080);
    f.setSize(1920, 1080);
    f.transformation = dai::ImgTransformation(1920, 1080);

    f.transformation.addFlipHorizontal();
    f.transformation.addRotation(90, {960, 540});
    f.transformation.addFlipHorizontal();

    REQUIRE(f.validateTransformations());

    dai::Point2f p{300, 400};
    auto p1 = f.remapPointToSource(p);
    auto p2 = f.remapPointFromSource(p1);

    REQUIRE_THAT(p2.x, Catch::Matchers::WithinAbs(p.x, 0.01));
    REQUIRE_THAT(p2.y, Catch::Matchers::WithinAbs(p.y, 0.01));
}

// -----------------------------------------------------------------------------
// Purpose:
//  Compares the custom getOuterRotatedRect implementation against OpenCV's
//  minAreaRect to ensure they produce similar results.
// -----------------------------------------------------------------------------
TEST_CASE("Get outer rect opencv comparison") {
    for(auto i = 0u; i < 10000u; ++i) {
        // Generate random rect
        float x = static_cast<float>(rand() % 1000);
        float y = static_cast<float>(rand() % 1000);
        float w = static_cast<float>(rand() % 500 + 1);
        float h = static_cast<float>(rand() % 500 + 1);
        float angle = static_cast<float>(rand() % 360);
        std::vector<std::array<float, 2>> pointsArr;
        std::vector<cv::Point2f> cvPointsArr;
        dai::RotatedRect rr{dai::Point2f{x + w / 2.0f, y + h / 2.0f}, dai::Size2f{w, h}, angle};
        for(const auto& p : rr.getPoints()) {
            pointsArr.push_back({p.x, p.y});
            cvPointsArr.emplace_back(p.x, p.y);
        }
        auto rrImpl = dai::impl::getOuterRotatedRect(pointsArr);
        auto rrCv = cv::minAreaRect(cvPointsArr);
        while(rrCv.angle >= 90.f) {
            rrCv.angle -= 90.f;
            std::swap(rrCv.size.width, rrCv.size.height);
        }
        REQUIRE_THAT(rrImpl.center.x, Catch::Matchers::WithinAbs(rrCv.center.x, 0.01));
        REQUIRE_THAT(rrImpl.center.y, Catch::Matchers::WithinAbs(rrCv.center.y, 0.01));
        REQUIRE_THAT(rrImpl.size.width, Catch::Matchers::WithinAbs(rrCv.size.width, 0.01));
        REQUIRE_THAT(rrImpl.size.height, Catch::Matchers::WithinAbs(rrCv.size.height, 0.01));
        REQUIRE_THAT(rrImpl.angle, Catch::Matchers::WithinAbs(rrCv.angle, 0.01));
    }
}
