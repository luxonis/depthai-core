#include <algorithm>
#include <catch2/catch_test_macros.hpp>
#include <cstdlib>
#include <vector>

#include "pipeline/utilities/Stitching/FixedBlender.hpp"

namespace {

dai::utilities::FixedBlender::Source flatSource(cv::Point tl, const cv::Mat& seamMask) {
    cv::Mat map(seamMask.size(), CV_32FC2);
    map.forEach<cv::Vec2f>([](cv::Vec2f& value, const int* position) { value = cv::Vec2f(static_cast<float>(position[1]), static_cast<float>(position[0])); });
    return {tl, cv::Mat(seamMask.size(), CV_8U, cv::Scalar(255)), seamMask, cv::Scalar::all(1.0), map, cv::Mat()};
}

}  // namespace

TEST_CASE("Fixed blender ramps across an overlap narrower than its band", "[Stitching]") {
    const cv::Size size(120, 16);
    cv::Mat leftSeam = cv::Mat::zeros(size, CV_8U);
    leftSeam.colRange(0, 110).setTo(255);
    cv::Mat rightSeam = cv::Mat::zeros(size, CV_8U);
    rightSeam.colRange(10, 120).setTo(255);

    dai::utilities::FixedBlender blender;
    blender.prepare(cv::Rect(0, 0, 220, 16), {flatSource({0, 0}, leftSeam), flatSource({100, 0}, rightSeam)}, 61, cv::BORDER_REFLECT);
    const cv::Mat result = blender.blend({cv::Mat(size, CV_8UC3, cv::Scalar::all(50)), cv::Mat(size, CV_8UC3, cv::Scalar::all(150))});

    REQUIRE(result.at<cv::Vec3b>(8, 0) == cv::Vec3b(50, 50, 50));
    REQUIRE(result.at<cv::Vec3b>(8, 219) == cv::Vec3b(150, 150, 150));
    int largestStep = 0;
    for(int x = 1; x < result.cols; ++x) {
        largestStep = std::max(largestStep, std::abs(result.at<cv::Vec3b>(8, x)[0] - result.at<cv::Vec3b>(8, x - 1)[0]));
    }
    REQUIRE(largestStep <= 15);
}

TEST_CASE("Fixed blender samples a source that contributes at both ends of the canvas", "[Stitching]") {
    const cv::Size size(300, 8);
    cv::Mat endsSeam = cv::Mat::zeros(size, CV_8U);
    endsSeam.colRange(0, 50).setTo(255);
    endsSeam.colRange(250, 300).setTo(255);
    const cv::Mat middleSeam = 255 - endsSeam;

    cv::Mat ramp(size, CV_8UC3);
    for(int x = 0; x < size.width; ++x) ramp.col(x).setTo(cv::Scalar::all(x * 255 / (size.width - 1)));

    dai::utilities::FixedBlender blender;
    blender.prepare(cv::Rect(0, 0, 300, 8), {flatSource({0, 0}, endsSeam), flatSource({0, 0}, middleSeam)}, 5, cv::BORDER_REFLECT);
    const cv::Mat result = blender.blend({ramp, cv::Mat(size, CV_8UC3, cv::Scalar::all(77))});
    for(const int x : {0, 20, 280, 299}) REQUIRE(result.at<cv::Vec3b>(4, x) == ramp.at<cv::Vec3b>(4, x));
    REQUIRE(result.at<cv::Vec3b>(4, 150) == cv::Vec3b(77, 77, 77));
}

TEST_CASE("Fixed blender limits the exposure gain of a far darker overlap", "[Stitching]") {
    const cv::Size size(100, 8);
    const cv::Mat full(size, CV_8U, cv::Scalar(255));
    cv::detail::NoSeamFinder noSeams;

    dai::utilities::FixedBlender blender;
    const std::vector<cv::Mat> images = {cv::Mat(size, CV_8UC3, cv::Scalar::all(2)), cv::Mat(size, CV_8UC3, cv::Scalar::all(120))};
    blender.prepareBlended(cv::Rect(0, 0, 150, 8), {flatSource({0, 0}, full), flatSource({50, 0}, full)}, images, noSeams, -1.0, cv::BORDER_REFLECT);
    const cv::Mat result = blender.blend(images);

    REQUIRE(result.at<cv::Vec3b>(4, 0)[0] <= 4);
    REQUIRE(result.at<cv::Vec3b>(4, 149)[0] >= 60);
}
