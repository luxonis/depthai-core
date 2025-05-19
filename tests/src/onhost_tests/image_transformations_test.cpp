// Add test case for every transformation + edge cases

#include "depthai/common/ImgTransformations.hpp"
#include "depthai/utility/ImageManipImpl.hpp"
#define CATCH_CONFIG_MAIN

#include <catch2/catch_all.hpp>
#include <chrono>
#include <iostream>
#include <thread>

#include "depthai/depthai.hpp"

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
