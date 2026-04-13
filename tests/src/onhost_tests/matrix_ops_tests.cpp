#include <array>
#include <catch2/catch_all.hpp>
#include <depthai/common/Point2f.hpp>
#include <depthai/utility/matrixOps.hpp>

#ifdef DEPTHAI_HAVE_OPENCV_SUPPORT
    #include <opencv2/imgproc.hpp>
#endif

namespace {

[[maybe_unused]] std::array<std::array<float, 3>, 3> normalizeHomography(const std::array<std::array<float, 3>, 3>& homography) {
    const float scale = homography[2][2];
    REQUIRE(scale != 0.0f);

    std::array<std::array<float, 3>, 3> normalized{};
    for(size_t row = 0; row < 3; ++row) {
        for(size_t col = 0; col < 3; ++col) {
            normalized[row][col] = homography[row][col] / scale;
        }
    }
    return normalized;
}

[[maybe_unused]] std::array<float, 2> transformPoint(const std::array<std::array<float, 3>, 3>& homography, const dai::Point2f& point) {
    const float x = homography[0][0] * point.x + homography[0][1] * point.y + homography[0][2];
    const float y = homography[1][0] * point.x + homography[1][1] * point.y + homography[1][2];
    const float z = homography[2][0] * point.x + homography[2][1] * point.y + homography[2][2];
    REQUIRE(z != 0.0f);
    return {x / z, y / z};
}

#ifdef DEPTHAI_HAVE_OPENCV_SUPPORT
std::array<std::array<float, 3>, 3> cvHomographyToArray(const cv::Mat& matrix) {
    std::array<std::array<float, 3>, 3> result{};
    for(int row = 0; row < 3; ++row) {
        for(int col = 0; col < 3; ++col) {
            result[row][col] = static_cast<float>(matrix.at<double>(row, col));
        }
    }
    return result;
}

void compareWithOpenCV(const std::array<dai::Point2f, 4>& srcPoints,
                       const std::array<dai::Point2f, 4>& dstPoints,
                       float epsilonMat = 1e-6f,
                       float epsilonPt = 1e-3f) {
    const auto homography = normalizeHomography(dai::matrix::getHomographyMatrix(srcPoints, dstPoints));

    cv::Point2f cvSrcPoints[4];
    cv::Point2f cvDstPoints[4];
    for(int i = 0; i < 4; ++i) {
        cvSrcPoints[i] = cv::Point2f(srcPoints[i].x, srcPoints[i].y);
        cvDstPoints[i] = cv::Point2f(dstPoints[i].x, dstPoints[i].y);
    }

    const auto cvHomography = normalizeHomography(cvHomographyToArray(cv::getPerspectiveTransform(cvSrcPoints, cvDstPoints)));

    for(size_t row = 0; row < 3; ++row) {
        for(size_t col = 0; col < 3; ++col) {
            REQUIRE_THAT(homography[row][col], Catch::Matchers::WithinAbs(cvHomography[row][col], epsilonMat));
        }
    }

    for(size_t i = 0; i < srcPoints.size(); ++i) {
        const auto transformed = transformPoint(homography, srcPoints[i]);
        REQUIRE_THAT(transformed[0], Catch::Matchers::WithinAbs(dstPoints[i].x, epsilonPt));
        REQUIRE_THAT(transformed[1], Catch::Matchers::WithinAbs(dstPoints[i].y, epsilonPt));
    }
}
#endif

}  // namespace

#ifdef DEPTHAI_HAVE_OPENCV_SUPPORT
TEST_CASE("Homography matches OpenCV for rectangular warp") {
    const std::array<dai::Point2f, 4> srcPoints = {
        dai::Point2f{0.0f, 0.0f}, dai::Point2f{640.0f, 0.0f}, dai::Point2f{640.0f, 480.0f}, dai::Point2f{0.0f, 480.0f}};
    const std::array<dai::Point2f, 4> dstPoints = {
        dai::Point2f{24.0f, 40.0f}, dai::Point2f{612.0f, 18.0f}, dai::Point2f{590.0f, 458.0f}, dai::Point2f{52.0f, 470.0f}};

    compareWithOpenCV(srcPoints, dstPoints);
}

TEST_CASE("Homography matches OpenCV for arbitrary quadrilateral warp") {
    const std::array<dai::Point2f, 4> srcPoints = {
        dai::Point2f{112.5f, 37.25f}, dai::Point2f{504.75f, 91.0f}, dai::Point2f{468.0f, 356.5f}, dai::Point2f{86.25f, 330.75f}};
    const std::array<dai::Point2f, 4> dstPoints = {
        dai::Point2f{8.0f, 16.0f}, dai::Point2f{420.0f, 4.0f}, dai::Point2f{448.0f, 284.0f}, dai::Point2f{36.0f, 300.0f}};

    compareWithOpenCV(srcPoints, dstPoints);

    const std::array<dai::Point2f, 4> srcPoints2 = {
        dai::Point2f(1830.f / 2, 244.f / 2), dai::Point2f(1846.f / 2, 208.f / 2), dai::Point2f{1875.f / 2, 208.f / 2}, dai::Point2f{1859.f / 2, 245.f / 2}};
    const std::array<dai::Point2f, 4> dstPoints2 = {dai::Point2f{0.f, 0.f}, dai::Point2f{219.f, 0.f}, dai::Point2f{219.f, 159.f}, dai::Point2f{0.f, 159.f}};

    compareWithOpenCV(srcPoints2, dstPoints2);
}

#else
TEST_CASE("Homography OpenCV comparison requires OpenCV support") {
    SUCCEED("OpenCV support is disabled.");
}
#endif
