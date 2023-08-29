// Add test case for every transformation + edge cases

#define CATCH_CONFIG_MAIN

#include <catch2/catch.hpp>
#include <chrono>
#include <iostream>
#include <thread>

#include "depthai/depthai.hpp"

int testFov() {
    dai::ImgFrame frame;
    frame.setSourceSize(1280, 800);
    frame.setSourceHFov(72.79);
    REQUIRE_THAT(82.0, Catch::Matchers::WithinAbs(frame.getSourceDFov(), 0.1));
    REQUIRE_THAT(72.79, Catch::Matchers::WithinAbs(frame.getSourceHFov(), 0.1));
    REQUIRE_THAT(49.47, Catch::Matchers::WithinAbs(frame.getSourceVFov(), 0.1));
    return 0;
}

int testPadding() {
    auto sourceImageFrame = std::make_shared<dai::ImgFrame>();
    sourceImageFrame->setWidth(1920);
    sourceImageFrame->setHeight(1080);
    sourceImageFrame->setSourceSize(1920, 1080);
    sourceImageFrame->transformations.addPadding(100, 200, 300, 400);
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

    sourceImageFrame->transformations.addCrop(x, y, x + width, y + height);
    sourceImageFrame->setSize(300, 400);
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
    sourceImageFrame->transformations.addFlipHorizontal();
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

    sourceImageFrame->transformations.addScale(12, 0.3);
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
    REQUIRE(sourceImageFrame->validateTransformations());

    sourceImageFrame->transformations.addRotation(angle, rotationPoint, sourceImageFrame->getWidth(), sourceImageFrame->getHeight());
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
    testRotation(dai::Point2f(960, 540), 0, dai::Point2f(960, 540));
    testRotation(dai::Point2f(0, 0), 0, dai::Point2f(0, 0));
    testRotation(dai::Point2f(1000, 1000), 0, dai::Point2f(1000, 1000));
    testRotation(dai::Point2f(420, 1080), -90, dai::Point2f(420, 0));
    testRotation(dai::Point2f(420, 1080), 90, dai::Point2f(1500, 1080));
    testRotation(dai::Point2f(1920, 0), 90, dai::Point2f(420, 0), dai::Point2f(960, 540), dai::Point2f(1500, 0));
    testRotation(dai::Point2f(0, 0), 90, dai::Point2f(0, 0), dai::Point2f(0, 0), dai::Point2f(0, 0));
}

TEST_CASE("testFOV") {
    testFov();
}
