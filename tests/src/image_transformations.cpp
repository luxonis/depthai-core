// Add test case for every transformation + edge cases

#define CATCH_CONFIG_MAIN

#include <catch2/catch.hpp>
#include <chrono>
#include <thread>
#include <iostream>

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
    sourceImageFrame->setSourceHeight(1080);
    sourceImageFrame->setSourceWidth(1920);
    sourceImageFrame->transformations.setPadding(100, 200, 300, 400);

    dai::Rect sourceRect{0.3, 0.7, 0.1, 0.1};
    auto outRect = sourceImageFrame->transformRectToSource(sourceRect);
    // std::cout << "x " << outRect.x << " y " << outRect.y << " width " << outRect.width << " height " << outRect.height << std::endl;
    REQUIRE_THAT(0.25, Catch::Matchers::WithinAbs(outRect.x, 0.01));
    REQUIRE_THAT(0.80, Catch::Matchers::WithinAbs(outRect.y, 0.01));
    REQUIRE_THAT(0.13, Catch::Matchers::WithinAbs(outRect.width, 0.01));
    REQUIRE_THAT(0.12, Catch::Matchers::WithinAbs(outRect.height, 0.01));
    auto reverseBackRect = sourceImageFrame->transformRectFromSource(outRect);
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
    sourceImageFrame->setSourceHeight(1080);
    sourceImageFrame->setSourceWidth(1920);
    float x = 0.44;
    float y = 0.2;
    float width = 0.3;
    float height = 0.7;

    sourceImageFrame->transformations.setCrop(x, y, x + width, y + width);
    // std::cout << "ImageHeight " << sourceImageFrame->getWidth() << std::endl;
    // std::cout << "ImageWidth " << sourceImageFrame->getHeight() << std::endl;

    dai::Rect sourceRect{0.3, 0.7, 0.1, 0.1};
    auto outRect = sourceImageFrame->transformRectToSource(sourceRect);
    // std::cout << "x " << outRect.x << " y " << outRect.y << " width " << outRect.width << " height " << outRect.height << std::endl;
    REQUIRE_THAT(0.53, Catch::Matchers::WithinAbs(outRect.x, 0.01));
    REQUIRE_THAT(0.69, Catch::Matchers::WithinAbs(outRect.y, 0.01));
    REQUIRE_THAT(0.03, Catch::Matchers::WithinAbs(outRect.width, 0.01));
    REQUIRE_THAT(0.07, Catch::Matchers::WithinAbs(outRect.height, 0.01));
    auto reverseBackRect = sourceImageFrame->transformRectFromSource(outRect);
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
    sourceImageFrame->setSourceHeight(1080);
    sourceImageFrame->setSourceWidth(1920);
    float x = 0.44;
    float y = 0.2;
    float width = 0.3;
    float height = 0.7;

    sourceImageFrame->transformations.setFlipHorizontal();
    sourceImageFrame->transformations.setFlipVertical();
    // std::cout << "ImageHeight " << sourceImageFrame->getWidth() << std::endl;
    // std::cout << "ImageWidth " << sourceImageFrame->getHeight() << std::endl;

    dai::Rect sourceRect{0.3, 0.7, 0.1, 0.1};
    auto outRect = sourceImageFrame->transformRectToSource(sourceRect);
    // std::cout << "x " << outRect.x << " y " << outRect.y << " width " << outRect.width << " height " << outRect.height << std::endl;
    REQUIRE_THAT(0.6, Catch::Matchers::WithinAbs(outRect.x, 0.01));
    REQUIRE_THAT(0.2, Catch::Matchers::WithinAbs(outRect.y, 0.01));
    REQUIRE_THAT(0.1, Catch::Matchers::WithinAbs(outRect.width, 0.01));
    REQUIRE_THAT(0.1, Catch::Matchers::WithinAbs(outRect.height, 0.01));
    auto reverseBackRect = sourceImageFrame->transformRectFromSource(outRect);
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
    sourceImageFrame->setSourceHeight(1080);
    sourceImageFrame->setSourceWidth(1920);
    float x = 0.44;
    float y = 0.2;
    float width = 0.3;
    float height = 0.7;

    sourceImageFrame->transformations.setCrop(12, 0.3);
    // std::cout << "ImageHeight " << sourceImageFrame->getWidth() << std::endl;
    // std::cout << "ImageWidth " << sourceImageFrame->getHeight() << std::endl;

    dai::Rect sourceRect{0.3, 0.7, 0.1, 0.1};
    auto outRect = sourceImageFrame->transformRectToSource(sourceRect);
    // std::cout << "x " << outRect.x << " y " << outRect.y << " width " << outRect.width << " height " << outRect.height << std::endl;
    REQUIRE_THAT(0.3, Catch::Matchers::WithinAbs(outRect.x, 0.01));
    REQUIRE_THAT(0.7, Catch::Matchers::WithinAbs(outRect.y, 0.01));
    REQUIRE_THAT(0.1, Catch::Matchers::WithinAbs(outRect.width, 0.01));
    REQUIRE_THAT(0.1, Catch::Matchers::WithinAbs(outRect.height, 0.01));
    auto reverseBackRect = sourceImageFrame->transformRectFromSource(outRect);
    REQUIRE_THAT(reverseBackRect.x, Catch::Matchers::WithinAbs(sourceRect.x, 0.01));
    REQUIRE_THAT(reverseBackRect.y, Catch::Matchers::WithinAbs(sourceRect.y, 0.01));
    REQUIRE_THAT(reverseBackRect.width, Catch::Matchers::WithinAbs(sourceRect.width, 0.01));
    REQUIRE_THAT(reverseBackRect.height, Catch::Matchers::WithinAbs(sourceRect.height, 0.01));
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

// TODO When implementation is done
// TEST_CASE("testRotation"){
// }

TEST_CASE("testFOV") {
    testFov();
}
