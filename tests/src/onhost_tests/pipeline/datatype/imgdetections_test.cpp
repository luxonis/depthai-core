#include <algorithm>
#include <array>
#include <catch2/catch_all.hpp>
#include <catch2/catch_message.hpp>
#include <catch2/catch_test_macros.hpp>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <limits>
#include <memory>
#include <numeric>
#include <optional>
#include <stdexcept>
#include <string>
#include <vector>

#ifdef DEPTHAI_HAVE_OPENCV_SUPPORT
    #include <opencv2/core.hpp>
    #include <opencv2/imgproc.hpp>
#endif

#include "depthai/common/Extrinsics.hpp"
#include "depthai/common/Keypoint.hpp"
#include "depthai/pipeline/MessageQueue.hpp"
#include "depthai/pipeline/Pipeline.hpp"
#include "depthai/pipeline/datatype/ImgDetections.hpp"
#include "depthai/pipeline/datatype/ImgFrame.hpp"
#include "depthai/pipeline/datatype/SpatialImgDetections.hpp"
#include "depthai/pipeline/datatype/Tracklets.hpp"
#include "depthai/pipeline/node/ObjectTracker.hpp"
#include "depthai/pipeline/node/SpatialLocationCalculator.hpp"
#include "depthai/utility/Serialization.hpp"

namespace {

std::vector<uint8_t> makeSequentialMask(std::size_t width, std::size_t height) {
    std::vector<uint8_t> mask(width * height);
    std::iota(mask.begin(), mask.end(), static_cast<uint8_t>(0));
    return mask;
}

}  // namespace

TEST_CASE("ImgDetection bounding box operations", "[ImgDetections][ImgDetection]") {
    using namespace dai;

    SECTION("setBoundingBox stores geometry and legacy coordinates") {
        ImgDetection detection;
        const RotatedRect rect{Point2f{0.4F, 0.6F, true}, Size2f{0.2F, 0.3F, true}, 30.0F};

        detection.setBoundingBox(rect);

        auto restored = detection.getBoundingBox();
        REQUIRE(restored.center.x == Catch::Approx(rect.center.x));
        REQUIRE(restored.center.y == Catch::Approx(rect.center.y));
        REQUIRE(restored.size.width == Catch::Approx(rect.size.width));
        REQUIRE(restored.size.height == Catch::Approx(rect.size.height));
        REQUIRE(restored.angle == Catch::Approx(rect.angle));

        auto outer = rect.getOuterRect();
        REQUIRE(detection.xmin == Catch::Approx(outer[0]));
        REQUIRE(detection.ymin == Catch::Approx(outer[1]));
        REQUIRE(detection.xmax == Catch::Approx(outer[2]));
        REQUIRE(detection.ymax == Catch::Approx(outer[3]));

        REQUIRE(detection.getCenterX() == Catch::Approx(rect.center.x));
        REQUIRE(detection.getCenterY() == Catch::Approx(rect.center.y));
        REQUIRE(detection.getWidth() == Catch::Approx(rect.size.width));
        REQUIRE(detection.getHeight() == Catch::Approx(rect.size.height));
        REQUIRE(detection.getAngle() == Catch::Approx(rect.angle));
    }

    SECTION("setOuterBoundingBox rebuilds bounding box") {
        ImgDetection detection;
        detection.setOuterBoundingBox(0.1F, 0.2F, 0.9F, 0.8F);

        auto bbox = detection.getBoundingBox();
        REQUIRE(bbox.center.x == Catch::Approx((0.1F + 0.9F) / 2.0F));
        REQUIRE(bbox.center.y == Catch::Approx((0.2F + 0.8F) / 2.0F));
        REQUIRE(bbox.size.width == Catch::Approx(0.9F - 0.1F));
        REQUIRE(bbox.size.height == Catch::Approx(0.8F - 0.2F));
        REQUIRE(bbox.angle == Catch::Approx(0.0F));
    }

    SECTION("getBoundingBox throws when no geometry was set") {
        ImgDetection detection;
        REQUIRE_THROWS_AS(detection.getBoundingBox(), std::runtime_error);
        REQUIRE_THROWS_AS(detection.getOuterBoundingBox(), std::runtime_error);
    }

    SECTION("getOuterBoundingBox accepts explicitly set zero bounds") {
        ImgDetection detection;
        detection.setOuterBoundingBox(0.f, 0.f, 0.f, 0.f);
        const std::array<float, 4> expected{0.f, 0.f, 0.f, 0.f};
        REQUIRE(detection.getOuterBoundingBox() == expected);
    }

    SECTION("getOuterBoundingBox uses a directly assigned bounding box") {
        const RotatedRect rect{Point2f{0.4F, 0.6F, true}, Size2f{0.2F, 0.3F, true}, 0.0F};
        const auto expected = rect.getOuterRect();

        ImgDetection detection;
        detection.boundingBox = rect;
        REQUIRE(detection.getOuterBoundingBox() == expected);

        SpatialImgDetection spatial;
        spatial.boundingBox = rect;
        const auto converted = spatial.getImgDetection();
        REQUIRE(converted.getOuterBoundingBox() == expected);
        REQUIRE(std::array<float, 4>{converted.xmin, converted.ymin, converted.xmax, converted.ymax} == expected);
    }
}

TEST_CASE("ImgDetection keypoints management", "[ImgDetections][Keypoints]") {
    using namespace dai;

    const std::vector<Keypoint> baseKeypoints{
        Keypoint(Point3f{0.0F, 1.0F, 2.0F}, 0.9F, 1), Keypoint(Point3f{1.0F, 2.0F, 3.0F}, 0.8F, 2), Keypoint(Point3f{2.0F, 3.0F, 4.0F}, 0.7F, 3)};
    const std::vector<Edge> edges{{0, 1}, {1, 2}};

    SECTION("setKeypoints from KeypointsList preserves edges") {
        KeypointsList list(baseKeypoints, edges);
        ImgDetection detection;

        detection.setKeypoints(list);
        auto restored = detection.getKeypoints();
        REQUIRE(restored.size() == baseKeypoints.size());
        for(std::size_t i = 0; i < restored.size(); ++i) {
            REQUIRE(restored[i].imageCoordinates.x == Catch::Approx(baseKeypoints[i].imageCoordinates.x));
            REQUIRE(restored[i].imageCoordinates.y == Catch::Approx(baseKeypoints[i].imageCoordinates.y));
            REQUIRE(restored[i].imageCoordinates.z == Catch::Approx(baseKeypoints[i].imageCoordinates.z));
            REQUIRE(restored[i].confidence == Catch::Approx(baseKeypoints[i].confidence));
        }
        REQUIRE(detection.getEdges() == edges);
    }

    SECTION("setKeypoints from std::vector<Keypoint> stores data") {
        ImgDetection detection;
        detection.setKeypoints(baseKeypoints);

        auto restored = detection.getKeypoints();
        REQUIRE(restored.size() == baseKeypoints.size());
        REQUIRE(detection.getEdges().empty());
    }

    SECTION("setKeypoints with edges validates indices") {
        ImgDetection detection;
        detection.setKeypoints(baseKeypoints);

        detection.setEdges(edges);
        REQUIRE(detection.getEdges() == edges);

        ImgDetection other;
        REQUIRE_THROWS_AS(other.setEdges(edges), std::runtime_error);
    }

    SECTION("setKeypoints with explicit edges") {
        ImgDetection detection;
        detection.setKeypoints(baseKeypoints, edges);

        REQUIRE(detection.getKeypoints().size() == baseKeypoints.size());
        REQUIRE(detection.getEdges() == edges);

        const std::vector<Edge> badEdges{{0, 3}};
        ImgDetection invalidDetection;
        REQUIRE_THROWS_AS(invalidDetection.setKeypoints(baseKeypoints, badEdges), std::invalid_argument);
    }

    SECTION("setKeypoints from Point3f vector initializes coordinates") {
        ImgDetection detection;
        std::vector<Point3f> coords{{0.0F, 1.0F, 2.0F}, {3.0F, 4.0F, 5.0F}};
        detection.setKeypoints(coords);

        auto restored = detection.getKeypoints();
        REQUIRE(restored.size() == coords.size());
        for(std::size_t i = 0; i < restored.size(); ++i) {
            REQUIRE(restored[i].imageCoordinates.x == Catch::Approx(coords[i].x));
            REQUIRE(restored[i].imageCoordinates.y == Catch::Approx(coords[i].y));
            REQUIRE(restored[i].imageCoordinates.z == Catch::Approx(coords[i].z));
        }
        REQUIRE(detection.getEdges().empty());
    }

    SECTION("setKeypoints from Point2f vector drops z coordinate") {
        ImgDetection detection;
        std::vector<Point2f> coords{{0.1F, 0.2F}, {0.3F, 0.4F}};
        detection.setKeypoints(coords);

        auto restored = detection.getKeypoints();
        REQUIRE(restored.size() == coords.size());
        REQUIRE(restored[0].imageCoordinates.x == Catch::Approx(coords[0].x));
        REQUIRE(restored[0].imageCoordinates.y == Catch::Approx(coords[0].y));
        REQUIRE(restored[0].imageCoordinates.z == Catch::Approx(0.0F));
        REQUIRE(detection.getEdges().empty());
    }
}

TEST_CASE("KeypointsList utilities", "[ImgDetections][KeypointsList]") {
    using namespace dai;

    const std::vector<Keypoint> keypoints{Keypoint(Point3f{0.0F, 0.5F, 1.0F}, 0.95F, 10), Keypoint(Point3f{1.0F, 1.5F, 2.0F}, 0.85F, 20)};

    SECTION("setKeypoints resets edges and exposes metadata") {
        KeypointsList list;
        list.setKeypoints(keypoints);

        auto restored = list.getKeypoints();
        REQUIRE(restored.size() == keypoints.size());
        REQUIRE(list.getEdges().empty());

        auto coords3 = list.getPoints3f();
        REQUIRE(coords3.size() == keypoints.size());
        REQUIRE(coords3[0].z == Catch::Approx(1.0F));

        auto coords2 = list.getPoints2f();
        REQUIRE(coords2.size() == keypoints.size());
        REQUIRE(coords2[0].x == Catch::Approx(0.0F));
        REQUIRE(coords2[0].y == Catch::Approx(0.5F));
    }

    SECTION("setEdges validates indices and prevents self loops") {
        KeypointsList list(keypoints);
        REQUIRE_NOTHROW(list.setEdges({Edge{0, 1}}));
        REQUIRE(list.getEdges().size() == 1);

        REQUIRE_THROWS_AS(list.setEdges({Edge{0, 0}}), std::invalid_argument);
        REQUIRE_THROWS_AS(list.setEdges({Edge{0, 2}}), std::invalid_argument);
    }

    SECTION("setKeypoints with edges validates input") {
        KeypointsList list;
        REQUIRE_NOTHROW(list.setKeypoints(keypoints, {Edge{0, 1}}));
        REQUIRE(list.getEdges().size() == 1);

        REQUIRE_THROWS_AS(list.setKeypoints(keypoints, {Edge{1, 1}}), std::invalid_argument);
    }

    SECTION("setKeypoints from coordinates clears previous edges") {
        KeypointsList list;
        list.setKeypoints(keypoints, {Edge{0, 1}});

        std::vector<Point3f> coords3{{2.0F, 3.0F, 4.0F}};
        list.setKeypoints(coords3);
        REQUIRE(list.getKeypoints().size() == 1);
        REQUIRE(list.getEdges().empty());

        std::vector<Point2f> coords2{{0.25F, 0.75F}};
        list.setKeypoints(coords2);
        REQUIRE(list.getKeypoints().size() == 1);
        REQUIRE(list.getKeypoints()[0].imageCoordinates.z == Catch::Approx(0.0F));
        REQUIRE(list.getEdges().empty());
    }
}

TEST_CASE("Keypoint validation", "[ImgDetections][Keypoint]") {
    using namespace dai;

    REQUIRE_NOTHROW(Keypoint(Point3f{0.0F, 0.0F, 0.0F}, 0.1F));
}

TEST_CASE("ImgDetections transformTo remaps detections and keypoints", "[ImgDetections][Transformations]") {
    using namespace dai;

    SECTION("transformTo throws when source transformation is missing") {
        ImgDetections detections;
        ImgTransformation target(100, 50);
        target.addScale(2.0F, 2.0F);

        REQUIRE_THROWS_AS(detections.transformTo(target), std::runtime_error);
    }

    SECTION("transformTo remaps bounding boxes and keypoints to resized target") {
        ImgDetections detections;
        ImgTransformation source(100, 50);
        ImgTransformation target(100, 50);
        Extrinsics extrinsics{{{1, 0, 0}, {0, 1, 0}, {0, 0, 1}}, {0, 0, 0}, CameraBoardSocket::CAM_A};
        source.setExtrinsics(extrinsics);
        target.setExtrinsics(extrinsics);
        target.addScale(2.0F, 2.0F);

        detections.setTransformation(source);

        ImgDetection detection;
        detection.setBoundingBox(RotatedRect{Point2f{20.0F, 10.0F}, Size2f{30.0F, 20.0F}, 0.0F});
        detection.setKeypoints(std::vector<Keypoint>{Keypoint(Point3f{10.0F, 15.0F, 7.0F}, 0.9F, 1), Keypoint(Point3f{40.0F, 5.0F, 9.0F}, 0.8F, 2)},
                               {Edge{0, 1}});
        detections.detections.push_back(detection);

        ImgDetections transformed = detections.transformTo(target);

        REQUIRE(transformed.getTransformation().has_value());
        REQUIRE(transformed.getTransformation()->getSize() == std::pair<size_t, size_t>{200, 100});
        REQUIRE(detections.getTransformation().has_value());
        REQUIRE(detections.getTransformation()->getSize() == std::pair<size_t, size_t>{100, 50});

        REQUIRE(transformed.detections.size() == 1);
        const auto& transformedDetection = transformed.detections.front();
        const auto transformedBox = transformedDetection.getBoundingBox();
        REQUIRE(transformedBox.center.x == Catch::Approx(40.0F));
        REQUIRE(transformedBox.center.y == Catch::Approx(20.0F));
        REQUIRE(transformedBox.size.width == Catch::Approx(60.0F));
        REQUIRE(transformedBox.size.height == Catch::Approx(40.0F));
        REQUIRE(transformedBox.angle == Catch::Approx(0.0F));

        const auto transformedKeypoints = transformedDetection.getKeypoints();
        REQUIRE(transformedKeypoints.size() == 2);
        REQUIRE(transformedKeypoints[0].imageCoordinates.x == Catch::Approx(20.0F));
        REQUIRE(transformedKeypoints[0].imageCoordinates.y == Catch::Approx(30.0F));
        REQUIRE(transformedKeypoints[0].imageCoordinates.z == Catch::Approx(7.0F));
        REQUIRE(transformedKeypoints[0].confidence == Catch::Approx(0.9F));
        REQUIRE(transformedKeypoints[1].imageCoordinates.x == Catch::Approx(80.0F));
        REQUIRE(transformedKeypoints[1].imageCoordinates.y == Catch::Approx(10.0F));
        REQUIRE(transformedKeypoints[1].imageCoordinates.z == Catch::Approx(9.0F));
        REQUIRE(transformedKeypoints[1].confidence == Catch::Approx(0.8F));
        REQUIRE(transformedDetection.getEdges() == std::vector<Edge>{Edge{0, 1}});
    }
}

TEST_CASE("ImgDetections segmentation mask operations", "[ImgDetections][Segmentation]") {
    using namespace dai;

    SECTION("Default construction exposes zero-sized mask") {
        const ImgDetections detections;
        REQUIRE(detections.getSegmentationMaskWidth() == 0);
        REQUIRE(detections.getSegmentationMaskHeight() == 0);
        REQUIRE(!detections.getMaskData());
    }

    SECTION("setMask stores dimensions and data") {
        ImgDetections detections;
        const std::size_t width = 4;
        const std::size_t height = 3;
        const auto mask = makeSequentialMask(width, height);

        detections.setSegmentationMask(mask, width, height);
        REQUIRE(detections.getSegmentationMaskWidth() == width);
        REQUIRE(detections.getSegmentationMaskHeight() == height);
        REQUIRE(detections.getMaskData() == mask);

        const auto sequenceNum = 42;
        const auto timestamp = std::chrono::steady_clock::time_point{std::chrono::milliseconds{1234}};
        const auto deviceTimestamp = std::chrono::steady_clock::time_point{std::chrono::milliseconds{5678}};
        detections.setSequenceNum(sequenceNum);
        detections.setTimestamp(timestamp);
        detections.setTimestampDevice(deviceTimestamp);

        std::optional<dai::ImgFrame> optFrame = detections.getSegmentationMask();
        REQUIRE(optFrame);
        dai::ImgFrame frame = *optFrame;
        REQUIRE(frame.getWidth() == width);
        REQUIRE(frame.getHeight() == height);
        REQUIRE(frame.getType() == ImgFrame::Type::GRAY8);
        REQUIRE(frame.getSequenceNum() == sequenceNum);
        REQUIRE(frame.getTimestamp() == timestamp);
        REQUIRE(frame.getTimestampDevice() == deviceTimestamp);

        auto spanData = frame.getData();
        REQUIRE(spanData.size() == mask.size());
        REQUIRE(std::equal(spanData.begin(), spanData.end(), mask.begin()));
    }

    SECTION("setMask rejects mismatched dimensions") {
        ImgDetections detections;
        std::vector<uint8_t> mask = {0, 1, 2};
        REQUIRE_THROWS_AS(detections.setSegmentationMask(mask, 4, 4), std::runtime_error);
    }

#ifdef DEPTHAI_HAVE_OPENCV_SUPPORT
    SECTION("OpenCV segmentation mask copy semantics") {
        ImgDetections detections;
        constexpr int rows = 3;
        constexpr int cols = 4;
        cv::Mat mask(rows, cols, CV_8UC1);
        for(int r = 0; r < rows; ++r) {
            for(int c = 0; c < cols; ++c) {
                mask.at<uint8_t>(r, c) = static_cast<uint8_t>(r * cols + c);
            }
        }

        detections.setCvSegmentationMask(mask);
        REQUIRE(detections.getSegmentationMaskWidth() == static_cast<std::size_t>(mask.cols));
        REQUIRE(detections.getSegmentationMaskHeight() == static_cast<std::size_t>(mask.rows));

        std::optional<cv::Mat> optShallow = detections.getCvSegmentationMask();
        REQUIRE(optShallow.has_value());
        cv::Mat shallow = *optShallow;
        cv::Mat diff;
        cv::absdiff(shallow, mask, diff);
        REQUIRE(cv::countNonZero(diff) == 0);

        shallow.at<uint8_t>(0, 0) = static_cast<uint8_t>(mask.at<uint8_t>(0, 0) + 10);

        auto optShallowData = detections.getMaskData();
        REQUIRE(optShallowData);
        auto shallowData = *optShallowData;

        REQUIRE_FALSE(shallowData.empty());
        REQUIRE(shallowData.front() == mask.at<uint8_t>(0, 0));
        REQUIRE(shallowData.front() != shallow.at<uint8_t>(0, 0));

        cv::Mat constantMask(rows, cols, CV_8UC1, cv::Scalar(7));
        detections.setCvSegmentationMask(constantMask);

        std::optional<cv::Mat> optDeep = detections.getCvSegmentationMask();
        REQUIRE(optDeep);
        cv::Mat deep = *optDeep;
        REQUIRE(cv::countNonZero(deep != constantMask) == 0);

        deep.at<uint8_t>(0, 0) = 42;
        auto optDeepData = detections.getMaskData();
        REQUIRE(optDeepData);
        auto deepData = *optDeepData;
        REQUIRE_FALSE(deepData.empty());
        REQUIRE(deepData.front() == constantMask.at<uint8_t>(0, 0));
    }

    SECTION("OpenCV segmentation mask copy and class extraction") {
        ImgDetections detections;
        constexpr int rows = 2;
        constexpr int cols = 5;
        const std::array<uint8_t, rows * cols> values{0, 1, 2, 1, 0, 2, 1, 0, 2, 1};
        cv::Mat mask(rows, cols, CV_8UC1);
        std::size_t idx = 0;
        for(int r = 0; r < rows; ++r) {
            for(int c = 0; c < cols; ++c) {
                mask.at<uint8_t>(r, c) = values[idx++];
            }
        }

        detections.setCvSegmentationMask(mask);
        std::optional<cv::Mat> optCopy = detections.getCvSegmentationMask();
        REQUIRE(optCopy.has_value());
        cv::Mat copy = *optCopy;
        cv::Mat diff;
        cv::absdiff(copy, mask, diff);
        REQUIRE(cv::countNonZero(diff) == 0);

        copy.at<uint8_t>(0, 0) = 99;
        auto optStoredData = detections.getMaskData();
        REQUIRE(optStoredData);
        auto storedData = *optStoredData;
        REQUIRE(storedData.front() == mask.at<uint8_t>(0, 0));

        constexpr uint8_t targetIndex = 1;
        std::optional<cv::Mat> optByIndex = detections.getCvSegmentationMaskByIndex(targetIndex);
        REQUIRE(optByIndex.has_value());
        cv::Mat byIndex = *optByIndex;
        REQUIRE(byIndex.rows == mask.rows);
        REQUIRE(byIndex.cols == mask.cols);
        REQUIRE(byIndex.type() == CV_8UC1);

        int expectedMatches = 0;
        for(int r = 0; r < rows; ++r) {
            for(int c = 0; c < cols; ++c) {
                if(mask.at<uint8_t>(r, c) == targetIndex) {
                    ++expectedMatches;
                }
            }
        }
        REQUIRE(cv::countNonZero(byIndex) == expectedMatches);
        if(expectedMatches > 0) {
            double minVal = 0.0;
            double maxVal = 0.0;
            cv::minMaxLoc(byIndex, &minVal, &maxVal);
            REQUIRE(minVal == 0.0);
            REQUIRE(maxVal == 255.0);
        }

        detections.detections.clear();
        detections.detections.resize(3);
        detections.detections[0].label = 5;
        detections.detections[1].label = 7;
        detections.detections[2].label = 5;

        std::optional<cv::Mat> optByClass = detections.getCvSegmentationMaskByClass(5);
        REQUIRE(optByClass.has_value());
        cv::Mat byClass = *optByClass;
        REQUIRE(byClass.rows == mask.rows);
        REQUIRE(byClass.cols == mask.cols);
        REQUIRE(byClass.type() == CV_8UC1);

        int expectedZeroPixels = 0;
        for(uint8_t value : values) {
            if(value == 0 || value == 2) {
                ++expectedZeroPixels;
            }
        }
        cv::Mat zeroMask;
        cv::compare(byClass, 0, zeroMask, cv::CmpTypes::CMP_EQ);
        REQUIRE(cv::countNonZero(zeroMask) == expectedZeroPixels);

        cv::Mat unexpected;
        cv::inRange(byClass, 1, 254, unexpected);
        REQUIRE(cv::countNonZero(unexpected) == 0);

        std::optional<cv::Mat> optMissingClass = detections.getCvSegmentationMaskByClass(99);
        REQUIRE(optMissingClass.has_value());
        cv::Mat missingClass = *optMissingClass;
        cv::Mat allBackground;
        cv::compare(missingClass, 255, allBackground, cv::CmpTypes::CMP_EQ);
        REQUIRE(cv::countNonZero(allBackground) == missingClass.rows * missingClass.cols);
    }
#endif
}

TEST_CASE("Transformed detection coordinates enclose the physical rotated box", "[ImgDetections][Transformations]") {
    const auto size = GENERATE(std::make_pair(800U, 500U), std::make_pair(500U, 800U));
    const auto angle = GENERATE(0.0f, 30.0f, 89.0f, 91.0f);
    const bool normalized = GENERATE(false, true);
    const bool swapSides = GENERATE(false, true);
    CAPTURE(size.first, size.second, angle, normalized, swapSides);

    dai::ImgTransformation source(size.first, size.second);
    source.setExtrinsics(dai::Extrinsics{{{1, 0, 0}, {0, 1, 0}, {0, 0, 1}}, {0, 0, 0}, dai::CameraBoardSocket::CAM_A});
    auto target = source;
    target.addScale(0.5f, 0.5f);
    target.addCrop(20, 10, 300, 200);
    const dai::Point2f center{size.first / 2.0f, size.second / 2.0f, false};
    const dai::RotatedRect physical{center, dai::Size2f{200, 100, false}, angle};
    const dai::RotatedRect equivalent{center, dai::Size2f{100, 200, false}, angle + 90};
    auto box = swapSides ? equivalent : physical;
    if(normalized) box = box.normalize(size.first, size.second);

    auto expected = physical.getOuterRect();
    // Uniform scale followed by a crop preserves the rectangle and its angle.
    expected[0] = expected[0] * 0.5f - 20;
    expected[1] = expected[1] * 0.5f - 10;
    expected[2] = expected[2] * 0.5f - 20;
    expected[3] = expected[3] * 0.5f - 10;
    const auto checkBounds = [&](const auto& detection) {
        const std::array<float, 4> bounds{detection.xmin, detection.ymin, detection.xmax, detection.ymax};
        for(std::size_t i = 0; i < bounds.size(); ++i) {
            const float scale = normalized ? (i % 2 == 0 ? 300.0f : 200.0f) : 1.0f;
            REQUIRE_THAT(bounds[i] * scale, Catch::Matchers::WithinAbs(expected[i], 1e-3f));
        }
        // Crop consumers must still receive the same physical rotated rectangle.
        const auto result = detection.getBoundingBox();
        REQUIRE(result.isNormalized() == normalized);
        const auto pixels = result.denormalize(300, 200);
        REQUIRE_THAT(pixels.size.width * pixels.size.height, Catch::Matchers::WithinAbs(5000.0f, 0.02f));
        const auto outer = pixels.getOuterRect();
        for(std::size_t i = 0; i < outer.size(); ++i) {
            REQUIRE_THAT(outer[i], Catch::Matchers::WithinAbs(expected[i], 1e-3f));
        }
    };

    const auto checkImgBounds = [&](const dai::ImgDetection& detection) {
        checkBounds(detection);
        const auto outer = detection.getOuterBoundingBox();
        for(std::size_t i = 0; i < outer.size(); ++i) {
            const float scale = normalized ? (i % 2 == 0 ? 300.0f : 200.0f) : 1.0f;
            REQUIRE_THAT(outer[i] * scale, Catch::Matchers::WithinAbs(expected[i], 1e-3f));
        }
    };

    dai::ImgDetections detections;
    detections.setTransformation(source);
    detections.detections.emplace_back(box);
    const auto transformed = detections.transformTo(target);
    checkImgBounds(transformed.detections.front());
    dai::ImgDetections restored;
    dai::utility::deserialize(dai::utility::serialize(transformed), restored);
    checkImgBounds(restored.detections.front());

    for(const float depth : {0.0f, 1000.0f}) {
        dai::SpatialImgDetection spatial(box, dai::Point3f{0, 0, depth});
        spatial.transform(source, target, dai::LengthUnit::MILLIMETER);
        checkBounds(spatial);
        checkImgBounds(spatial.getImgDetection());
        dai::SpatialImgDetection restoredSpatial;
        dai::utility::deserialize(dai::utility::serialize(spatial), restoredSpatial);
        checkImgBounds(restoredSpatial.getImgDetection());
    }
}

TEST_CASE("Detections keep their size and angle through a small rotation", "[ImgDetections][Tracklets][Transformations]") {
    // A rotation of -0.001 deg made the fitter return 89.999 deg with swapped sides.
    const float rotation = GENERATE(-0.001f, 0.001f);
    CAPTURE(rotation);

    dai::ImgTransformation source(800, 500);
    source.setExtrinsics(dai::Extrinsics{{{1, 0, 0}, {0, 1, 0}, {0, 0, 1}}, {0, 0, 0}, dai::CameraBoardSocket::CAM_A});
    auto target = source;
    target.addRotation(rotation, dai::Point2f{400, 250, false});
    const dai::RotatedRect pixels{dai::Point2f{400, 250, false}, dai::Size2f{200, 100, false}, 0.0f};
    const auto box = pixels.normalize(800, 500);
    // The rotation moves the corners by less than 0.01 px.
    const auto expected = pixels.getOuterRect();
    const std::array<float, 4> scale{800, 500, 800, 500};
    const auto checkBounds = [&](const std::array<float, 4>& bounds) {
        for(std::size_t i = 0; i < bounds.size(); ++i) {
            REQUIRE_THAT(bounds[i] * scale[i], Catch::Matchers::WithinAbs(expected[i], 0.1));
        }
    };

    dai::ImgDetections detections;
    detections.setTransformation(source);
    detections.detections.emplace_back(box);
    const auto detection = detections.transformTo(target).detections.front();
    REQUIRE_THAT(detection.getAngle(), Catch::Matchers::WithinAbs(rotation, 1e-2));
    REQUIRE_THAT(detection.getWidth() * 800, Catch::Matchers::WithinAbs(200, 0.1));
    REQUIRE_THAT(detection.getHeight() * 500, Catch::Matchers::WithinAbs(100, 0.1));
    checkBounds(detection.getOuterBoundingBox());
    // A detection made again from the box (as SpatialLocationCalculator does) has the same bounds.
    checkBounds(dai::ImgDetection(detection.getBoundingBox()).getOuterBoundingBox());

    dai::Tracklets tracklets;
    tracklets.setTransformation(source);
    dai::Tracklet tracklet;
    tracklet.roi = dai::Rect{dai::Point2f{expected[0] / 800, expected[1] / 500, true}, dai::Point2f{expected[2] / 800, expected[3] / 500, true}, true};
    tracklet.srcImgDetection = dai::ImgDetection(box);
    tracklets.tracklets.push_back(tracklet);
    const auto transformed = tracklets.transformTo(target).tracklets.front();
    const auto& roi = transformed.roi;
    REQUIRE(roi.isNormalized());
    checkBounds({roi.x, roi.y, roi.x + roi.width, roi.y + roi.height});
    checkBounds(transformed.srcImgDetection.getOuterBoundingBox());
}

namespace {

dai::ImgTransformation makeDetectionTransformation(unsigned int width, unsigned int height) {
    dai::ImgTransformation transformation(width, height);
    transformation.setExtrinsics(dai::Extrinsics{{{1, 0, 0}, {0, 1, 0}, {0, 0, 1}}, {0, 0, 0}, dai::CameraBoardSocket::CAM_A});
    return transformation;
}

void requireBounds(const std::array<float, 4>& bounds, const std::array<float, 4>& expected, const std::array<float, 4>& scale, float margin) {
    for(std::size_t i = 0; i < bounds.size(); ++i) {
        REQUIRE_THAT(bounds[i] * scale[i], Catch::Matchers::WithinAbs(expected[i], margin));
    }
}

}  // namespace

TEST_CASE("setBoundingBox with the frame size calculates the bounds in pixels", "[ImgDetections][ImgDetection][SpatialImgDetections]") {
    const dai::RotatedRect pixels{dai::Point2f{400, 250, false}, dai::Size2f{200, 100, false}, 30.0f};
    const auto normalized = pixels.normalize(800, 500);
    const auto expected = pixels.getOuterRect();
    const std::array<float, 4> scale{800, 500, 800, 500};

    dai::ImgDetection detection;
    detection.setBoundingBox(normalized, 800, 500);
    requireBounds(detection.getOuterBoundingBox(), expected, scale, 1e-3f);
    dai::SpatialImgDetection spatial;
    spatial.setBoundingBox(normalized, 800, 500);
    requireBounds(spatial.getOuterBoundingBox(), expected, scale, 1e-3f);

    // A pixel box does not use the frame size.
    detection.setBoundingBox(pixels, 800, 500);
    REQUIRE(detection.getOuterBoundingBox() == expected);

    // Without the frame size, the bounds are calculated in normalized units (the documented limit).
    detection.setBoundingBox(normalized);
    REQUIRE(detection.getOuterBoundingBox() == normalized.getOuterRect());
}

TEST_CASE("A box made from legacy bounds has explicit units", "[ImgDetections][ImgDetection][SpatialImgDetections]") {
    const auto source = makeDetectionTransformation(800, 500);
    auto target = source;
    target.addScale(0.5f, 0.5f);
    const auto transformBounds = [&](const dai::ImgDetection& detection) {
        dai::ImgDetections detections;
        detections.setTransformation(source);
        detections.detections.push_back(detection);
        return detections.transformTo(target).detections.front().getOuterBoundingBox();
    };

    SECTION("bounds of 1 or less are normalized") {
        dai::ImgDetection detection;
        detection.xmin = 0.2f;
        detection.ymin = 0.3f;
        detection.xmax = 0.6f;
        detection.ymax = 0.7f;
        const auto box = detection.getBoundingBox();
        REQUIRE(box.center.hasNormalized);
        REQUIRE(box.size.hasNormalized);
        REQUIRE(box.isNormalized());
        REQUIRE_THAT(box.center.x, Catch::Matchers::WithinAbs(0.4, 1e-6));
        REQUIRE_THAT(box.center.y, Catch::Matchers::WithinAbs(0.5, 1e-6));
        REQUIRE_THAT(box.size.width, Catch::Matchers::WithinAbs(0.4, 1e-6));
        REQUIRE_THAT(box.size.height, Catch::Matchers::WithinAbs(0.4, 1e-6));
        // A scale does not change normalized bounds.
        requireBounds(transformBounds(detection), {0.2f, 0.3f, 0.6f, 0.7f}, {1, 1, 1, 1}, 1e-5f);

        dai::SpatialImgDetection spatial;
        spatial.xmin = 0.2f;
        spatial.ymin = 0.3f;
        spatial.xmax = 0.6f;
        spatial.ymax = 0.7f;
        REQUIRE(spatial.getBoundingBox().center.hasNormalized);
        REQUIRE(spatial.getBoundingBox().isNormalized());
    }

    SECTION("a bound above 1 means pixels") {
        dai::ImgDetection detection;
        detection.xmin = 100.0f;
        detection.ymin = 50.0f;
        detection.xmax = 300.0f;
        detection.ymax = 250.0f;
        const auto box = detection.getBoundingBox();
        REQUIRE(box.center.hasNormalized);
        REQUIRE_FALSE(box.isNormalized());
        requireBounds(transformBounds(detection), {50.0f, 25.0f, 150.0f, 125.0f}, {1, 1, 1, 1}, 1e-3f);

        dai::ImgDetection outside;
        outside.xmin = 0.5f;
        outside.ymin = 0.4f;
        outside.xmax = 1.02f;
        outside.ymax = 0.6f;
        REQUIRE_FALSE(outside.getBoundingBox().isNormalized());

        dai::SpatialImgDetection spatial;
        spatial.xmin = 100.0f;
        spatial.ymin = 50.0f;
        spatial.xmax = 300.0f;
        spatial.ymax = 250.0f;
        REQUIRE(spatial.getBoundingBox().center.hasNormalized);
        REQUIRE_FALSE(spatial.getBoundingBox().isNormalized());
    }

    SECTION("setOuterBoundingBox sets explicit units") {
        dai::ImgDetection detection;
        detection.setOuterBoundingBox(0.2f, 0.3f, 0.6f, 0.7f);
        REQUIRE(detection.getBoundingBox().center.hasNormalized);
        REQUIRE(detection.getBoundingBox().isNormalized());
        detection.setOuterBoundingBox(100.0f, 50.0f, 300.0f, 250.0f);
        REQUIRE(detection.getBoundingBox().center.hasNormalized);
        REQUIRE_FALSE(detection.getBoundingBox().isNormalized());

        dai::SpatialImgDetection spatial;
        spatial.setOuterBoundingBox(0.2f, 0.3f, 0.6f, 0.7f);
        REQUIRE(spatial.getBoundingBox().center.hasNormalized);
        REQUIRE(spatial.getBoundingBox().isNormalized());
    }
}

namespace {

using Point = std::array<float, 2>;

// Maps a pixel point with the 3x3 matrix of a test transform.
Point mapPoint(const std::array<std::array<float, 3>, 3>& matrix, Point point) {
    return {matrix[0][0] * point[0] + matrix[0][1] * point[1] + matrix[0][2], matrix[1][0] * point[0] + matrix[1][1] * point[1] + matrix[1][2]};
}

// The smallest rectangle that encloses a convex quadrilateral has one side on an edge of the quadrilateral.
float minEnclosingArea(const std::array<Point, 4>& points) {
    float best = std::numeric_limits<float>::infinity();
    for(std::size_t i = 0; i < points.size(); ++i) {
        const Point& a = points[i];
        const Point& b = points[(i + 1) % points.size()];
        const float length = std::hypot(b[0] - a[0], b[1] - a[1]);
        const Point u{(b[0] - a[0]) / length, (b[1] - a[1]) / length};
        float minU = std::numeric_limits<float>::infinity(), maxU = -minU, minV = minU, maxV = -minU;
        for(const auto& p : points) {
            const float pu = p[0] * u[0] + p[1] * u[1];
            const float pv = -p[0] * u[1] + p[1] * u[0];
            minU = std::min(minU, pu);
            maxU = std::max(maxU, pu);
            minV = std::min(minV, pv);
            maxV = std::max(maxV, pv);
        }
        best = std::min(best, (maxU - minU) * (maxV - minV));
    }
    return best;
}

bool containsPoint(const dai::RotatedRect& rect, const Point& point, float margin) {
    const float angle = rect.angle * 3.14159265358979323846f / 180.0f;
    const float dx = point[0] - rect.center.x;
    const float dy = point[1] - rect.center.y;
    const float along = dx * std::cos(angle) + dy * std::sin(angle);
    const float across = -dx * std::sin(angle) + dy * std::cos(angle);
    return std::abs(along) <= rect.size.width / 2 + margin && std::abs(across) <= rect.size.height / 2 + margin;
}

// Checks that box is the minimum outer rotated rectangle of the mapped corners, and that bounds enclose box.
void requireMinimumOuterRect(
    const dai::RotatedRect& box, const std::array<float, 4>& bounds, const std::array<Point, 4>& corners, unsigned int width, unsigned int height) {
    const auto pixels = box.denormalize(width, height);
    REQUIRE_THAT(pixels.size.width * pixels.size.height, Catch::Matchers::WithinRel(minEnclosingArea(corners), 1e-3f));
    for(const auto& corner : corners) {
        REQUIRE(containsPoint(pixels, corner, 0.05f));
    }
    const bool normalized = box.isNormalized();
    const std::array<float, 4> scale =
        normalized ? std::array<float, 4>{static_cast<float>(width), static_cast<float>(height), static_cast<float>(width), static_cast<float>(height)}
                   : std::array<float, 4>{1, 1, 1, 1};
    requireBounds(bounds, pixels.getOuterRect(), scale, 1e-2f);
}

}  // namespace

TEST_CASE("Transformed detections use the minimum outer rotated rectangle", "[ImgDetections][SpatialImgDetections][Transformations]") {
    // A stretch or a shear changes a rotated box into a parallelogram. The bounding box must be the minimum outer rotated
    // rectangle of the parallelogram, and the bounds must enclose that rectangle.
    const auto [name, matrix] = GENERATE(table<std::string, std::array<std::array<float, 3>, 3>>({
        {"scale (1, 0.625)", {{{1.0f, 0.0f, 0.0f}, {0.0f, 0.625f, 0.0f}, {0.0f, 0.0f, 1.0f}}}},
        {"shear x += 0.5 * y", {{{1.0f, 0.5f, -125.0f}, {0.0f, 1.0f, 0.0f}, {0.0f, 0.0f, 1.0f}}}},
    }));
    const bool normalized = GENERATE(false, true);
    CAPTURE(name, normalized);

    const auto source = makeDetectionTransformation(800, 500);
    auto target = source;
    target.addTransformation(matrix);
    const auto targetSize = target.getSize();
    const auto width = static_cast<unsigned int>(targetSize.first);
    const auto height = static_cast<unsigned int>(targetSize.second);

    const dai::RotatedRect pixels{dai::Point2f{400, 250, false}, dai::Size2f{200, 100, false}, 30.0f};
    std::array<Point, 4> corners{};
    const auto sourceCorners = pixels.getPoints();
    for(std::size_t i = 0; i < corners.size(); ++i) {
        corners[i] = mapPoint(matrix, {sourceCorners[i].x, sourceCorners[i].y});
    }
    const auto box = normalized ? pixels.normalize(800, 500) : pixels;

    dai::ImgDetections detections;
    detections.setTransformation(source);
    detections.detections.emplace_back(box);
    const auto transformed = detections.transformTo(target).detections.front();
    REQUIRE(transformed.getBoundingBox().isNormalized() == normalized);
    requireMinimumOuterRect(transformed.getBoundingBox(), transformed.getOuterBoundingBox(), corners, width, height);

    for(const float depth : {0.0f, 1000.0f}) {
        CAPTURE(depth);
        dai::SpatialImgDetection spatial(box, dai::Point3f{0, 0, depth});
        spatial.transform(source, target, dai::LengthUnit::MILLIMETER);
        requireMinimumOuterRect(spatial.getBoundingBox(), spatial.getOuterBoundingBox(), corners, width, height);
        requireMinimumOuterRect(spatial.getBoundingBox(), spatial.getImgDetection().getOuterBoundingBox(), corners, width, height);
    }
}

TEST_CASE("Tracklet roi encloses the minimum outer rotated rectangle", "[Tracklets][Transformations]") {
    // A rotation followed by a stretch changes the roi into a parallelogram.
    const bool normalized = GENERATE(false, true);
    CAPTURE(normalized);
    const auto source = makeDetectionTransformation(800, 800);
    auto target = source;
    target.addRotation(30.0f, dai::Point2f{400, 400, false});
    target.addScale(1.0f, 0.625f);  // 800 x 500

    const dai::Rect pixels{dai::Point2f{300, 350, false}, dai::Point2f{500, 450, false}, false};
    const auto roi = normalized ? pixels.normalize(800, 800) : pixels;
    const std::array<float, 4> scale = normalized ? std::array<float, 4>{800, 500, 800, 500} : std::array<float, 4>{1, 1, 1, 1};

    dai::Tracklets tracklets;
    tracklets.setTransformation(source);
    dai::Tracklet tracklet;
    tracklet.roi = roi;
    tracklet.srcImgDetection = dai::ImgDetection(dai::RotatedRect(roi));
    tracklets.tracklets.push_back(tracklet);
    const auto result = tracklets.transformTo(target).tracklets.front().roi;
    REQUIRE(result.isNormalized() == normalized);

    // The roi is the outer rectangle of the minimum outer rotated rectangle (the remapped roi).
    const auto fitted = source.remapRectTo(target, dai::RotatedRect(roi)).denormalize(800, 500);
    requireBounds({result.x, result.y, result.x + result.width, result.y + result.height}, fitted.getOuterRect(), scale, 1e-2f);
}

TEST_CASE("SpatialImgDetection getOuterBoundingBox", "[SpatialImgDetections]") {
    const dai::RotatedRect rect{dai::Point2f{0.4F, 0.6F, true}, dai::Size2f{0.2F, 0.3F, true}, 0.0F};

    SECTION("returns the bounds set by setBoundingBox") {
        dai::SpatialImgDetection spatial;
        spatial.setBoundingBox(rect);
        REQUIRE(spatial.getOuterBoundingBox() == rect.getOuterRect());
    }

    SECTION("uses a directly assigned bounding box") {
        dai::SpatialImgDetection spatial;
        spatial.boundingBox = rect;
        REQUIRE(spatial.getOuterBoundingBox() == rect.getOuterRect());
    }

    SECTION("returns legacy bounds") {
        dai::SpatialImgDetection spatial;
        spatial.xmin = 0.1f;
        spatial.ymin = 0.2f;
        spatial.xmax = 0.3f;
        spatial.ymax = 0.4f;
        REQUIRE(spatial.getOuterBoundingBox() == std::array<float, 4>{0.1f, 0.2f, 0.3f, 0.4f});
    }

    SECTION("returns explicitly set zero bounds") {
        dai::SpatialImgDetection spatial;
        spatial.setOuterBoundingBox(0.f, 0.f, 0.f, 0.f);
        REQUIRE(spatial.getOuterBoundingBox() == std::array<float, 4>{0.f, 0.f, 0.f, 0.f});
    }

    SECTION("throws when no geometry was set") {
        dai::SpatialImgDetection spatial;
        REQUIRE_THROWS_AS(spatial.getOuterBoundingBox(), std::runtime_error);
    }
}

TEST_CASE("Legacy bounds that are all zero are not set", "[ImgDetections][ImgDetection][SpatialImgDetections]") {
    const dai::RotatedRect rect{dai::Point2f{0.4F, 0.6F, true}, dai::Size2f{0.2F, 0.3F, true}, 0.0F};
    dai::ImgDetection detection;
    REQUIRE(detection.xmin == 0.f);
    REQUIRE(detection.ymin == 0.f);
    REQUIRE(detection.xmax == 0.f);
    REQUIRE(detection.ymax == 0.f);
    REQUIRE_THROWS_AS(detection.getBoundingBox(), std::runtime_error);
    REQUIRE_THROWS_AS(detection.getOuterBoundingBox(), std::runtime_error);

    dai::SpatialImgDetection spatial;
    REQUIRE_THROWS_AS(spatial.getBoundingBox(), std::runtime_error);
    REQUIRE_THROWS_AS(spatial.getOuterBoundingBox(), std::runtime_error);

    SECTION("zero bounds with a bounding box use the bounding box") {
        detection.boundingBox = rect;
        REQUIRE(detection.getOuterBoundingBox() == rect.getOuterRect());
        spatial.boundingBox = rect;
        REQUIRE(spatial.getOuterBoundingBox() == rect.getOuterRect());
        REQUIRE(spatial.getImgDetection().getOuterBoundingBox() == rect.getOuterRect());
    }

    SECTION("bounds with one value that is not 0 are set") {
        detection.xmin = 0.1f;
        REQUIRE_NOTHROW(detection.getBoundingBox());
        REQUIRE(detection.getOuterBoundingBox() == std::array<float, 4>{0.1f, 0.f, 0.f, 0.f});
    }

    SECTION("serialization keeps the bounds that are not set") {
        dai::ImgDetections message;
        message.detections.push_back(detection);
        dai::ImgDetections restored;
        dai::utility::deserialize(dai::utility::serialize(message), restored);
        REQUIRE_THROWS_AS(restored.detections.front().getBoundingBox(), std::runtime_error);
    }
}

#ifdef DEPTHAI_ENABLE_PROTOBUF
TEST_CASE("Protobuf keeps transformed rotated detections", "[ImgDetections][SpatialImgDetections][Protobuf]") {
    const auto source = makeDetectionTransformation(800, 500);
    auto target = source;
    target.addRotation(30.0f, dai::Point2f{400, 250, false});
    const auto box = dai::RotatedRect{dai::Point2f{400, 250, false}, dai::Size2f{200, 100, false}, 0.0f}.normalize(800, 500);
    const auto requireSameBox = [](const dai::RotatedRect& lhs, const dai::RotatedRect& rhs) {
        REQUIRE(lhs.isNormalized() == rhs.isNormalized());
        REQUIRE(lhs.center.x == rhs.center.x);
        REQUIRE(lhs.center.y == rhs.center.y);
        REQUIRE(lhs.size.width == rhs.size.width);
        REQUIRE(lhs.size.height == rhs.size.height);
        REQUIRE(lhs.angle == rhs.angle);
    };

    dai::ImgDetections detections;
    detections.setTransformation(source);
    detections.detections.emplace_back(box);
    auto transformed = detections.transformTo(target);
    dai::ImgDetection boxOnly;
    boxOnly.boundingBox = box;
    transformed.detections.push_back(boxOnly);

    dai::ImgDetections restored;
    restored.deserializeProto(transformed.serializeProto());
    REQUIRE(restored.detections.size() == 2);
    REQUIRE(restored.detections[0].getOuterBoundingBox() == transformed.detections[0].getOuterBoundingBox());
    requireSameBox(restored.detections[0].getBoundingBox(), transformed.detections[0].getBoundingBox());
    REQUIRE(restored.detections[1].xmin == 0.f);
    requireSameBox(restored.detections[1].getBoundingBox(), box);
    REQUIRE(restored.detections[1].getOuterBoundingBox() == box.getOuterRect());

    dai::SpatialImgDetections spatialDetections;
    spatialDetections.setTransformation(source);
    spatialDetections.detections.emplace_back(box, dai::Point3f{0, 0, 1000});
    const auto spatialTransformed = spatialDetections.transformTo(target);
    dai::SpatialImgDetections spatialRestored;
    spatialRestored.deserializeProto(spatialTransformed.serializeProto());
    REQUIRE(spatialRestored.detections.size() == 1);
    REQUIRE(spatialRestored.detections[0].getOuterBoundingBox() == spatialTransformed.detections[0].getOuterBoundingBox());
    requireSameBox(spatialRestored.detections[0].getBoundingBox(), spatialTransformed.detections[0].getBoundingBox());
}
#endif

namespace {

constexpr std::chrono::seconds NODE_OUTPUT_TIMEOUT{10};

// Axis-aligned 200x100 px box in an 800x500 frame, transformed to a frame that is rotated by 30 deg.
// The outer bounds of the transformed box are 223.21 x 186.60 px.
std::shared_ptr<dai::ImgDetections> makeRotatedDetections(const dai::ImgTransformation& source, const dai::ImgTransformation& target) {
    dai::ImgDetections detections;
    detections.setTransformation(source);
    detections.detections.emplace_back(dai::RotatedRect{dai::Point2f{400, 250, false}, dai::Size2f{200, 100, false}, 0.0f}.normalize(800, 500), 1.0f, 0);
    auto transformed = std::make_shared<dai::ImgDetections>(detections.transformTo(target));
    transformed->setSequenceNum(1);
    return transformed;
}

std::shared_ptr<dai::ImgFrame> makeFrame(const dai::ImgTransformation& transformation, dai::ImgFrame::Type type, std::vector<std::uint8_t> data) {
    auto frame = std::make_shared<dai::ImgFrame>();
    const auto size = transformation.getSize();
    frame->setWidth(static_cast<unsigned int>(size.first));
    frame->setHeight(static_cast<unsigned int>(size.second));
    frame->setType(type);
    frame->setData(data);
    frame->transformation = transformation;
    frame->setSequenceNum(1);
    return frame;
}

template <typename Output>
std::shared_ptr<Output> getNodeOutput(dai::Pipeline& pipeline, const std::shared_ptr<dai::MessageQueue>& queue) {
    bool timedOut = false;
    auto output = queue->get<Output>(NODE_OUTPUT_TIMEOUT, timedOut);
    pipeline.stop();
    pipeline.wait();
    if(timedOut || output == nullptr) {
        throw std::runtime_error("Timed out waiting for the node output");
    }
    return output;
}

}  // namespace

TEST_CASE("SpatialLocationCalculator on host keeps the bounds of rotated detections", "[SpatialLocationCalculator][Transformations]") {
    const auto source = makeDetectionTransformation(800, 500);
    auto target = source;
    target.addRotation(30.0f, dai::Point2f{400, 250, false});
    const auto detections = makeRotatedDetections(source, target);
    const auto& input = detections->detections.front();

    // Depth frame of 1000 mm, aligned with the detections.
    std::vector<std::uint16_t> depth(800 * 500, 1000);
    std::vector<std::uint8_t> bytes(depth.size() * sizeof(std::uint16_t));
    std::memcpy(bytes.data(), depth.data(), bytes.size());
    const auto depthFrame = makeFrame(target, dai::ImgFrame::Type::RAW16, std::move(bytes));

    dai::Pipeline pipeline(false);
    auto calculator = pipeline.create<dai::node::SpatialLocationCalculator>();
    calculator->setRunOnHost(true);
    auto depthQueue = calculator->inputDepth.createInputQueue();
    auto detectionsQueue = calculator->inputDetections.createInputQueue();
    auto outputQueue = calculator->outputDetections.createOutputQueue();
    pipeline.start();
    depthQueue->send(depthFrame);
    detectionsQueue->send(detections);
    const auto output = getNodeOutput<dai::SpatialImgDetections>(pipeline, outputQueue);

    REQUIRE(output->detections.size() == 1);
    const auto& spatial = output->detections.front();
    const std::array<float, 4> unit{1, 1, 1, 1};
    // The output has the bounds of the input, which are calculated in pixels.
    const auto inputBounds = input.getOuterBoundingBox();
    requireBounds(spatial.getOuterBoundingBox(), inputBounds, unit, 1e-6f);
    REQUIRE_THAT((inputBounds[2] - inputBounds[0]) * 800, Catch::Matchers::WithinAbs(223.21, 0.01));
    REQUIRE_THAT((inputBounds[3] - inputBounds[1]) * 500, Catch::Matchers::WithinAbs(186.60, 0.01));
    // The depth ROI is the outer rectangle of the box in pixels of the detections frame.
    const auto& roi = spatial.boundingBoxMapping.roi;
    requireBounds({roi.x, roi.y, roi.x + roi.width, roi.y + roi.height}, input.getBoundingBox().getOuterRectInFrame(800, 500), unit, 1e-5f);
    REQUIRE_THAT(spatial.spatialCoordinates.z, Catch::Matchers::WithinAbs(1000.0, 1.0));
}

#ifdef DEPTHAI_HAVE_OPENCV_SUPPORT
namespace {

// Runs the tracker on host for one frame and returns the roi of the first tracklet in pixels of the tracker frame.
std::array<float, 4> trackOnce(const dai::ImgTransformation& trackerTransformation, const std::shared_ptr<dai::ImgDetections>& detections) {
    const auto size = trackerTransformation.getSize();
    const auto frame = makeFrame(trackerTransformation, dai::ImgFrame::Type::NV12, std::vector<std::uint8_t>(size.first * size.second * 3 / 2, 0));

    dai::Pipeline pipeline(false);
    auto tracker = pipeline.create<dai::node::ObjectTracker>();
    tracker->setRunOnHost(true);
    auto frameQueue = tracker->inputTrackerFrame.createInputQueue();
    auto detectionsQueue = tracker->inputDetections.createInputQueue();
    auto outputQueue = tracker->out.createOutputQueue();
    pipeline.start();
    frameQueue->send(frame);
    detectionsQueue->send(detections);
    const auto tracklets = getNodeOutput<dai::Tracklets>(pipeline, outputQueue);

    REQUIRE(tracklets->tracklets.size() == 1);
    const auto roi = tracklets->tracklets.front().roi.denormalize(static_cast<int>(size.first), static_cast<int>(size.second));
    return {roi.x, roi.y, roi.x + roi.width, roi.y + roi.height};
}

}  // namespace

TEST_CASE("ObjectTracker on host tracks the bounds of rotated detections", "[ObjectTracker][Transformations]") {
    const auto source = makeDetectionTransformation(800, 500);
    auto target = source;
    target.addRotation(30.0f, dai::Point2f{400, 250, false});
    const auto detections = makeRotatedDetections(source, target);
    const auto bounds = detections->detections.front().getOuterBoundingBox();
    const std::array<float, 4> unit{1, 1, 1, 1};
    // The tracker works in whole pixels.
    const float margin = 1.5f;

    SECTION("tracker frame is the detections frame") {
        requireBounds(trackOnce(target, detections), {bounds[0] * 800, bounds[1] * 500, bounds[2] * 800, bounds[3] * 500}, unit, margin);
    }

    SECTION("tracker frame is the frame before the rotation") {
        // The outer bounds (223.21 x 186.60 px) turned back by 30 deg give an outer rectangle of 286.6 x 273.2 px.
        const float width = 223.21f * 0.8660254f + 186.60f * 0.5f;
        const float height = 223.21f * 0.5f + 186.60f * 0.8660254f;
        requireBounds(trackOnce(source, detections), {400 - width / 2, 250 - height / 2, 400 + width / 2, 250 + height / 2}, unit, margin);
    }
}

TEST_CASE("ObjectTracker on host uses the units of the detection box", "[ObjectTracker]") {
    // A box of 1.2 x 1.2 px. The tracker must not read it as normalized because it is small.
    const auto transformation = makeDetectionTransformation(800, 500);
    auto detections = std::make_shared<dai::ImgDetections>();
    detections->setTransformation(transformation);
    detections->detections.emplace_back(dai::RotatedRect{dai::Point2f{400.6f, 250.6f, false}, dai::Size2f{1.2f, 1.2f, false}, 0.0f}, 1.0f, 0);
    detections->setSequenceNum(1);

    const auto roi = trackOnce(transformation, detections);
    REQUIRE(roi[2] - roi[0] <= 2.0f);
    REQUIRE(roi[3] - roi[1] <= 2.0f);
    requireBounds(roi, {400.0f, 250.0f, 401.2f, 251.2f}, {1, 1, 1, 1}, 1.5f);
}
#endif
