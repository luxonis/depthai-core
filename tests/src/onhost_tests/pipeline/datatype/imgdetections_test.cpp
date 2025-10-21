#include <algorithm>
#include <array>
#include <catch2/catch_all.hpp>
#include <catch2/catch_message.hpp>
#include <catch2/catch_test_macros.hpp>
#include <chrono>
#include <numeric>
#include <stdexcept>
#include <string>
#include <vector>

#ifdef DEPTHAI_HAVE_OPENCV_SUPPORT
    #include <opencv2/core.hpp>
    #include <opencv2/imgproc.hpp>
#endif

#include "depthai/common/Keypoint.hpp"
#include "depthai/pipeline/datatype/ImgDetections.hpp"
#include "depthai/pipeline/datatype/ImgFrame.hpp"

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
    }
}

TEST_CASE("ImgDetection keypoints management", "[ImgDetections][Keypoints]") {
    using namespace dai;

    const std::vector<Keypoint> baseKeypoints{Keypoint(Point3f{0.0F, 1.0F, 2.0F}, 0.9F, 1, "nose"),
                                              Keypoint(Point3f{1.0F, 2.0F, 3.0F}, 0.8F, 2, "eye"),
                                              Keypoint(Point3f{2.0F, 3.0F, 4.0F}, 0.7F, 3, "ear")};
    const std::vector<Edge> edges{{0, 1}, {1, 2}};

    SECTION("setKeypoints from KeypointsList preserves edges") {
        KeypointsList list(baseKeypoints, edges);
        ImgDetection detection;

        detection.setKeypoints(list);
        auto restored = detection.getKeypoints();
        REQUIRE(restored.size() == baseKeypoints.size());
        for(std::size_t i = 0; i < restored.size(); ++i) {
            REQUIRE(restored[i].coordinates.x == Catch::Approx(baseKeypoints[i].coordinates.x));
            REQUIRE(restored[i].coordinates.y == Catch::Approx(baseKeypoints[i].coordinates.y));
            REQUIRE(restored[i].coordinates.z == Catch::Approx(baseKeypoints[i].coordinates.z));
            REQUIRE(restored[i].confidence == Catch::Approx(baseKeypoints[i].confidence));
            REQUIRE(restored[i].label == baseKeypoints[i].label);
            REQUIRE(restored[i].labelName == baseKeypoints[i].labelName);
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
            REQUIRE(restored[i].coordinates.x == Catch::Approx(coords[i].x));
            REQUIRE(restored[i].coordinates.y == Catch::Approx(coords[i].y));
            REQUIRE(restored[i].coordinates.z == Catch::Approx(coords[i].z));
        }
        REQUIRE(detection.getEdges().empty());
    }

    SECTION("setKeypoints from Point2f vector drops z coordinate") {
        ImgDetection detection;
        std::vector<Point2f> coords{{0.1F, 0.2F}, {0.3F, 0.4F}};
        detection.setKeypoints(coords);

        auto restored = detection.getKeypoints();
        REQUIRE(restored.size() == coords.size());
        REQUIRE(restored[0].coordinates.x == Catch::Approx(coords[0].x));
        REQUIRE(restored[0].coordinates.y == Catch::Approx(coords[0].y));
        REQUIRE(restored[0].coordinates.z == Catch::Approx(0.0F));
        REQUIRE(detection.getEdges().empty());
    }
}

TEST_CASE("KeypointsList utilities", "[ImgDetections][KeypointsList]") {
    using namespace dai;

    const std::vector<Keypoint> keypoints{Keypoint(Point3f{0.0F, 0.5F, 1.0F}, 0.95F, 10, "left"), Keypoint(Point3f{1.0F, 1.5F, 2.0F}, 0.85F, 20, "right")};

    SECTION("setKeypoints resets edges and exposes metadata") {
        KeypointsList list;
        list.setKeypoints(keypoints);

        auto restored = list.getKeypoints();
        REQUIRE(restored.size() == keypoints.size());
        REQUIRE(list.getEdges().empty());

        auto coords3 = list.getCoordinates3f();
        REQUIRE(coords3.size() == keypoints.size());
        REQUIRE(coords3[0].z == Catch::Approx(1.0F));

        auto coords2 = list.getCoordinates2f();
        REQUIRE(coords2.size() == keypoints.size());
        REQUIRE(coords2[0].x == Catch::Approx(0.0F));
        REQUIRE(coords2[0].y == Catch::Approx(0.5F));

        auto labels = list.getLabels();
        REQUIRE(labels == std::vector<std::string>{"left", "right"});
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
        REQUIRE(list.getKeypoints()[0].coordinates.z == Catch::Approx(0.0F));
        REQUIRE(list.getEdges().empty());
    }
}

TEST_CASE("Keypoint validation", "[ImgDetections][Keypoint]") {
    using namespace dai;

    REQUIRE_NOTHROW(Keypoint(Point3f{0.0F, 0.0F, 0.0F}, 0.1F));
    REQUIRE_THROWS_AS(Keypoint(Point3f{0.0F, 0.0F, 0.0F}, -0.1F), std::invalid_argument);
}

TEST_CASE("ImgDetections segmentation mask operations", "[ImgDetections][Segmentation]") {
    using namespace dai;

    SECTION("Default construction exposes zero-sized mask") {
        const ImgDetections detections;
        REQUIRE(detections.getSegmentationMaskWidth() == 0);
        REQUIRE(detections.getSegmentationMaskHeight() == 0);
        REQUIRE(detections.getMaskData().empty());
    }

    SECTION("setMask stores dimensions and data") {
        ImgDetections detections;
        const std::size_t width = 4;
        const std::size_t height = 3;
        const auto mask = makeSequentialMask(width, height);

        detections.setMask(mask, width, height);
        REQUIRE(detections.getSegmentationMaskWidth() == width);
        REQUIRE(detections.getSegmentationMaskHeight() == height);
        REQUIRE(detections.getMaskData() == mask);

        const auto sequenceNum = 42;
        const auto timestamp = std::chrono::steady_clock::time_point{std::chrono::milliseconds{1234}};
        const auto deviceTimestamp = std::chrono::steady_clock::time_point{std::chrono::milliseconds{5678}};
        detections.setSequenceNum(sequenceNum);
        detections.setTimestamp(timestamp);
        detections.setTimestampDevice(deviceTimestamp);

        auto frame = detections.getSegmentationMaskAsImgFrame();
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
        REQUIRE_THROWS_AS(detections.setMask(mask, 4, 4), std::runtime_error);
    }

#ifdef DEPTHAI_HAVE_OPENCV_SUPPORT
    SECTION("OpenCV segmentation mask view semantics") {
        ImgDetections detections;
        constexpr int rows = 3;
        constexpr int cols = 4;
        cv::Mat mask(rows, cols, CV_8UC1);
        for(int r = 0; r < rows; ++r) {
            for(int c = 0; c < cols; ++c) {
                mask.at<uint8_t>(r, c) = static_cast<uint8_t>(r * cols + c);
            }
        }

        detections.setSegmentationMask(mask);
        REQUIRE(detections.getSegmentationMaskWidth() == static_cast<std::size_t>(mask.cols));
        REQUIRE(detections.getSegmentationMaskHeight() == static_cast<std::size_t>(mask.rows));

        cv::Mat shallow = detections.getSegmentationMask(false);
        cv::Mat diff;
        cv::absdiff(shallow, mask, diff);
        REQUIRE(cv::countNonZero(diff) == 0);

        shallow.at<uint8_t>(0, 0) = static_cast<uint8_t>(mask.at<uint8_t>(0, 0) + 10);
        auto shallowData = detections.getMaskData();
        REQUIRE_FALSE(shallowData.empty());
        REQUIRE(shallowData.front() == shallow.at<uint8_t>(0, 0));

        cv::Mat constantMask(rows, cols, CV_8UC1, cv::Scalar(7));
        detections.setSegmentationMask(constantMask);

        cv::Mat deep = detections.getSegmentationMask(true);
        REQUIRE(cv::countNonZero(deep != constantMask) == 0);

        deep.at<uint8_t>(0, 0) = 42;
        auto deepData = detections.getMaskData();
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

        detections.setSegmentationMask(mask);
        cv::Mat copy = detections.getCvSegmentationMask();
        cv::Mat diff;
        cv::absdiff(copy, mask, diff);
        REQUIRE(cv::countNonZero(diff) == 0);

        copy.at<uint8_t>(0, 0) = 99;
        auto storedData = detections.getMaskData();
        REQUIRE_FALSE(storedData.empty());
        REQUIRE(storedData.front() == mask.at<uint8_t>(0, 0));

        constexpr uint8_t targetIndex = 1;
        cv::Mat byIndex = detections.getCvSegmentationMaskByIndex(targetIndex);
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
    }
#endif
}
