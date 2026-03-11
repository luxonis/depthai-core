#include <catch2/catch_all.hpp>
#include <catch2/catch_test_macros.hpp>
#include <depthai/common/Rect.hpp>

#include "depthai/common/ImgTransformations.hpp"
#include "depthai/common/RotatedRect.hpp"
#ifdef DEPTHAI_HAVE_OPENCV_SUPPORT
    #include <opencv2/calib3d.hpp>
#endif

std::array<float, 3> undistortPoint(std::array<float, 3> point, dai::CameraModel model, const std::vector<float>& coeffs);

#ifdef DEPTHAI_HAVE_OPENCV_SUPPORT
dai::Point2f opencvPointTransformation(dai::Point2f sourcePt, const dai::ImgTransformation& from, const dai::ImgTransformation& to);

TEST_CASE("Perspective undistortPoint matches OpenCV at image edges") {
    const cv::Matx33f kIdentity(1.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 1.0f);
    const std::vector<std::vector<float>> distortionSets = {
        {0.11f, -0.24f, 0.0013f, -0.0005f, 0.13f, 0.003f, -0.001f, 0.0015f, -0.0003f, 0.0002f, -0.0001f, 0.0001f, -0.004f, 0.006f},
        {0.08f, -0.14f, 0.0009f, -0.0006f, 0.04f},
    };

    const std::vector<cv::Point2f> undistortedNormalized = {
        {0.0f, 0.0f},      {0.7f, 0.45f},    {-0.7f, 0.45f},   {0.7f, -0.45f},  {-0.7f, -0.45f},
        {0.85f, 0.55f},    {-0.85f, 0.55f},  {0.85f, -0.55f},  {-0.85f, -0.55f},
    };

    for(const auto& dist : distortionSets) {
        cv::Mat distCv(1, static_cast<int>(dist.size()), CV_32F);
        for(size_t i = 0; i < dist.size(); ++i) distCv.at<float>(0, static_cast<int>(i)) = dist[i];

        for(const auto& u : undistortedNormalized) {
            std::vector<cv::Point3f> objectPoints = {cv::Point3f(u.x, u.y, 1.0f)};
            std::vector<cv::Point2f> distortedPixels;
            cv::projectPoints(objectPoints, cv::Vec3f(0.0f, 0.0f, 0.0f), cv::Vec3f(0.0f, 0.0f, 0.0f), kIdentity, distCv, distortedPixels);

            std::vector<cv::Point2f> cvUndistorted;
            cv::undistortPoints(distortedPixels, cvUndistorted, kIdentity, distCv, cv::noArray(), cv::noArray());

            const auto ours = undistortPoint({distortedPixels[0].x, distortedPixels[0].y, 1.0f}, dai::CameraModel::Perspective, dist);
            REQUIRE_THAT(ours[0], Catch::Matchers::WithinAbs(cvUndistorted[0].x, 1e-4f));
            REQUIRE_THAT(ours[1], Catch::Matchers::WithinAbs(cvUndistorted[0].y, 1e-4f));
        }
    }
}

TEST_CASE("Fisheye undistortPoint matches OpenCV at image edges") {
    const std::vector<float> dist = {0.012f, -0.003f, 0.0004f, -0.00008f};
    cv::Mat distCv(1, static_cast<int>(dist.size()), CV_32F);
    for(size_t i = 0; i < dist.size(); ++i) distCv.at<float>(0, static_cast<int>(i)) = dist[i];

    const cv::Matx33f kIdentity(1.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 1.0f);

    const std::vector<cv::Point2f> undistortedNormalized = {
        {0.0f, 0.0f},      {0.7f, 0.45f},    {-0.7f, 0.45f},   {0.7f, -0.45f},  {-0.7f, -0.45f},
        {0.9f, 0.65f},     {-0.9f, 0.65f},   {0.9f, -0.65f},   {-0.9f, -0.65f},
    };

    for(const auto& u : undistortedNormalized) {
        std::vector<cv::Point2f> distortedPixels;
        cv::fisheye::distortPoints(std::vector<cv::Point2f>{u}, distortedPixels, kIdentity, distCv);

        std::vector<cv::Point2f> cvUndistorted;
        cv::fisheye::undistortPoints(distortedPixels, cvUndistorted, kIdentity, distCv, cv::noArray(), cv::noArray());

        const auto ours = undistortPoint({distortedPixels[0].x, distortedPixels[0].y, 1.0f}, dai::CameraModel::Fisheye, dist);
        REQUIRE_THAT(ours[0], Catch::Matchers::WithinAbs(cvUndistorted[0].x, 1e-5f));
        REQUIRE_THAT(ours[1], Catch::Matchers::WithinAbs(cvUndistorted[0].y, 1e-5f));
    }
}

TEST_CASE("opencvPointTransformation handles perspective source model") {
    auto makeIdentityExtrinsics = []() {
        dai::Extrinsics extr;
        extr.rotationMatrix = {{1.0f, 0.0f, 0.0f}, {0.0f, 1.0f, 0.0f}, {0.0f, 0.0f, 1.0f}};
        extr.translation = dai::Point3f(0.0f, 0.0f, 0.0f);
        extr.specTranslation = dai::Point3f(0.0f, 0.0f, 0.0f);
        extr.toCameraSocket = dai::CameraBoardSocket::AUTO;
        extr.lengthUnit = dai::LengthUnit::CENTIMETER;
        return extr;
    };

    const std::array<std::array<float, 3>, 3> intrinsics = {{{910.0f, 0.0f, 640.0f}, {0.0f, 910.0f, 400.0f}, {0.0f, 0.0f, 1.0f}}};
    const std::vector<float> fromDist = {0.11f, -0.24f, 0.0013f, -0.0005f, 0.13f, 0.003f, -0.001f, 0.0015f, -0.0003f, 0.0002f, -0.0001f, 0.0001f, -0.004f, 0.006f};
    const std::vector<float> toDist = {-0.02f, 0.03f, -0.0008f, 0.0004f, -0.01f, 0.002f, 0.001f, -0.001f, 0.0f, 0.0f, 0.0f, 0.0f, 0.002f, -0.003f};

    dai::ImgTransformation from(1280, 800);
    from.setIntrinsicMatrix(intrinsics);
    from.setDistortionModel(dai::CameraModel::Perspective);
    from.setDistortionCoefficients(fromDist);
    from.setExtrinsics(makeIdentityExtrinsics());

    dai::ImgTransformation to(1280, 800);
    to.setIntrinsicMatrix(intrinsics);
    to.setDistortionModel(dai::CameraModel::Perspective);
    to.setDistortionCoefficients(toDist);
    to.setExtrinsics(makeIdentityExtrinsics());

    const std::vector<dai::Point2f> points = {
        {640.0f, 400.0f},
        {420.0f, 300.0f},
        {980.0f, 300.0f},
        {420.0f, 650.0f},
        {980.0f, 650.0f},
    };

    for(const auto& p : points) {
        const auto expected = from.remapPointTo(to, p);
        const auto actual = opencvPointTransformation(p, from, to);
        REQUIRE_THAT(actual.x, Catch::Matchers::WithinAbs(expected.x, 0.05f));
        REQUIRE_THAT(actual.y, Catch::Matchers::WithinAbs(expected.y, 0.05f));
    }
}

TEST_CASE("projectPoint follows source-ray to target mapping for zero-baseline") {
    auto makeIdentityExtrinsics = []() {
        dai::Extrinsics extr;
        extr.rotationMatrix = {{1.0f, 0.0f, 0.0f}, {0.0f, 1.0f, 0.0f}, {0.0f, 0.0f, 1.0f}};
        extr.translation = dai::Point3f(0.0f, 0.0f, 0.0f);
        extr.specTranslation = dai::Point3f(0.0f, 0.0f, 0.0f);
        extr.toCameraSocket = dai::CameraBoardSocket::AUTO;
        extr.lengthUnit = dai::LengthUnit::CENTIMETER;
        return extr;
    };

    const std::array<std::array<float, 3>, 3> intrinsics = {{{910.0f, 0.0f, 640.0f}, {0.0f, 910.0f, 400.0f}, {0.0f, 0.0f, 1.0f}}};
    const std::vector<float> toDist = {0.11f, -0.24f, 0.0013f, -0.0005f, 0.13f, 0.003f, -0.001f, 0.0015f, -0.0003f, 0.0002f, -0.0001f, 0.0001f, -0.004f, 0.006f};

    dai::ImgTransformation from(1280, 800);
    from.setIntrinsicMatrix(intrinsics);
    from.setDistortionModel(dai::CameraModel::Perspective);
    from.setDistortionCoefficients(std::vector<float>(14, 0.0f));  // undistorted source stream
    from.setExtrinsics(makeIdentityExtrinsics());

    dai::ImgTransformation to(1280, 800);
    to.setIntrinsicMatrix(intrinsics);
    to.setDistortionModel(dai::CameraModel::Perspective);
    to.setDistortionCoefficients(toDist);  // distorted target stream
    to.setExtrinsics(makeIdentityExtrinsics());

    const std::vector<dai::Point2f> points = {
        {640.0f, 400.0f},
        {420.0f, 300.0f},
        {980.0f, 300.0f},
        {420.0f, 650.0f},
        {980.0f, 650.0f},
    };
    const std::vector<float> depthsMm = {500.0f, 1000.0f, 2000.0f};

    for(const auto& p : points) {
        const auto remapped = from.remapPointTo(to, p);
        for(float depth : depthsMm) {
            dai::Point2f pCopy = p;
            const auto projected = from.projectPoint(to, pCopy, depth);
            REQUIRE_THAT(projected.x, Catch::Matchers::WithinAbs(remapped.x, 0.05f));
            REQUIRE_THAT(projected.y, Catch::Matchers::WithinAbs(remapped.y, 0.05f));
        }
    }
}
#endif
bool float_eq(float a, float b) {
    return std::abs(a - b) < 1e-6;
}

TEST_CASE("Rect normalization") {
    dai::Rect rect(100, 100, 600, 400);
    REQUIRE(!rect.isNormalized());
    rect = rect.normalize(2000, 1000);
    REQUIRE(rect.isNormalized());
    REQUIRE(float_eq(rect.x, 0.05));
    REQUIRE(float_eq(rect.y, 0.1));
    REQUIRE(float_eq(rect.width, 0.3));
    REQUIRE(float_eq(rect.height, 0.4));
    rect = rect.denormalize(2000, 1000);
    REQUIRE(!rect.isNormalized());
    REQUIRE(rect.x == 100);
    REQUIRE(rect.y == 100);
    REQUIRE(rect.width == 600);
    REQUIRE(rect.height == 400);
}

TEST_CASE("RotatedRect normalization") {
    dai::RotatedRect rect(dai::Rect(100 - 300, 100 - 200, 600, 400), 45);
    REQUIRE(!rect.isNormalized());
    rect = rect.normalize(2000, 1000);
    REQUIRE(rect.isNormalized());
    REQUIRE(float_eq(rect.center.x, 0.05));
    REQUIRE(float_eq(rect.center.y, 0.1));
    REQUIRE(float_eq(rect.size.width, 0.3));
    REQUIRE(float_eq(rect.size.height, 0.4));
    rect = rect.denormalize(2000, 1000);
    REQUIRE(!rect.isNormalized());
    REQUIRE(rect.center.x == 100);
    REQUIRE(rect.center.y == 100);
    REQUIRE(rect.size.width == 600);
    REQUIRE(rect.size.height == 400);
}

TEST_CASE("Normalized point remap") {
    dai::Point2f pt(0.2, 0.3, true);
    REQUIRE(pt.isNormalized());
    dai::ImgTransformation transformFrom(2000, 1000);
    dai::ImgTransformation transformTo(2000, 1000);
    transformTo.addScale(2, 2);
    transformTo.addCrop(0, 0, 2000, 1000);
    pt = transformFrom.remapPointTo(transformTo, pt);
    REQUIRE(pt.isNormalized());
    REQUIRE(float_eq(pt.x, 0.4));
    REQUIRE(float_eq(pt.y, 0.6));
}

TEST_CASE("Normalized rect remap") {
    dai::RotatedRect rect(dai::Point2f(0.2, 0.3, true), dai::Size2f(0.4, 0.5, true), 45);
    REQUIRE(rect.isNormalized());
    dai::ImgTransformation transformFrom(2000, 1000);
    dai::ImgTransformation transformTo(2000, 1000);
    transformTo.addScale(2, 2);
    transformTo.addCrop(0, 0, 2000, 1000);
    rect = transformFrom.remapRectTo(transformTo, rect);
    REQUIRE(rect.isNormalized());
    REQUIRE(float_eq(rect.center.x, 0.4));
    REQUIRE(float_eq(rect.center.y, 0.6));
    REQUIRE(float_eq(rect.size.width, 0.8));
    REQUIRE(float_eq(rect.size.height, 1.0));
}

TEST_CASE("Remap undistorted to distorted applies target distortion") {
    auto distortPerspective = [](std::array<float, 3> ray, const std::vector<float>& coeffs) {
        const float x = ray[0];
        const float y = ray[1];
        const float k1 = coeffs.size() > 0 ? coeffs[0] : 0.0f;
        const float k2 = coeffs.size() > 1 ? coeffs[1] : 0.0f;
        const float p1 = coeffs.size() > 2 ? coeffs[2] : 0.0f;
        const float p2 = coeffs.size() > 3 ? coeffs[3] : 0.0f;
        const float k3 = coeffs.size() > 4 ? coeffs[4] : 0.0f;
        const float r2 = x * x + y * y;
        const float r4 = r2 * r2;
        const float r6 = r4 * r2;
        const float radial = 1.0f + k1 * r2 + k2 * r4 + k3 * r6;
        const float xTan = 2.0f * p1 * x * y + p2 * (r2 + 2.0f * x * x);
        const float yTan = p1 * (r2 + 2.0f * y * y) + 2.0f * p2 * x * y;
        return std::array<float, 3>{x * radial + xTan, y * radial + yTan, 1.0f};
    };

    auto makeIdentityExtrinsics = []() {
        dai::Extrinsics extr;
        extr.rotationMatrix = {{1.0f, 0.0f, 0.0f}, {0.0f, 1.0f, 0.0f}, {0.0f, 0.0f, 1.0f}};
        extr.translation = dai::Point3f(0.0f, 0.0f, 0.0f);
        extr.specTranslation = dai::Point3f(0.0f, 0.0f, 0.0f);
        extr.toCameraSocket = dai::CameraBoardSocket::AUTO;
        extr.lengthUnit = dai::LengthUnit::CENTIMETER;
        return extr;
    };

    const std::array<std::array<float, 3>, 3> intrinsics = {{{900.0f, 0.0f, 640.0f}, {0.0f, 900.0f, 400.0f}, {0.0f, 0.0f, 1.0f}}};
    const std::vector<float> toDistCoeffs = {0.12f, -0.05f, 0.001f, -0.001f, 0.01f};

    dai::ImgTransformation from(1280, 800);
    from.setIntrinsicMatrix(intrinsics);
    from.setDistortionModel(dai::CameraModel::Perspective);
    from.setDistortionCoefficients(std::vector<float>(5, 0.0f));
    from.setExtrinsics(makeIdentityExtrinsics());

    dai::ImgTransformation to(1280, 800);
    to.setIntrinsicMatrix(intrinsics);
    to.setDistortionModel(dai::CameraModel::Perspective);
    to.setDistortionCoefficients(toDistCoeffs);
    to.setExtrinsics(makeIdentityExtrinsics());

    const dai::Point2f sourcePoint(980.0f, 650.0f, false);
    const dai::Point2f mappedPoint = from.remapPointTo(to, sourcePoint);

    const std::array<float, 3> pxHomogeneous = {sourcePoint.x, sourcePoint.y, 1.0f};
    std::array<float, 3> undistortedRay = dai::matrix::matVecMul(from.getSourceIntrinsicMatrixInv(), pxHomogeneous);
    undistortedRay = {undistortedRay[0] / undistortedRay[2], undistortedRay[1] / undistortedRay[2], 1.0f};
    const std::array<float, 3> distortedRay = distortPerspective(undistortedRay, toDistCoeffs);
    const std::array<float, 3> expectedHomogeneous = dai::matrix::matVecMul(to.getSourceIntrinsicMatrix(), distortedRay);
    const float expectedX = expectedHomogeneous[0] / expectedHomogeneous[2];
    const float expectedY = expectedHomogeneous[1] / expectedHomogeneous[2];

    REQUIRE_THAT(mappedPoint.x, Catch::Matchers::WithinAbs(expectedX, 0.05f));
    REQUIRE_THAT(mappedPoint.y, Catch::Matchers::WithinAbs(expectedY, 0.05f));
}

#ifdef DEPTHAI_HAVE_OPENCV_SUPPORT
TEST_CASE("NonOpenCV remap matches OpenCV point transform") {
    auto makeIdentityExtrinsics = []() {
        dai::Extrinsics extr;
        extr.rotationMatrix = {{1.0f, 0.0f, 0.0f}, {0.0f, 1.0f, 0.0f}, {0.0f, 0.0f, 1.0f}};
        extr.translation = dai::Point3f(0.0f, 0.0f, 0.0f);
        extr.specTranslation = dai::Point3f(0.0f, 0.0f, 0.0f);
        extr.toCameraSocket = dai::CameraBoardSocket::AUTO;
        extr.lengthUnit = dai::LengthUnit::CENTIMETER;
        return extr;
    };

    const std::array<std::array<float, 3>, 3> intrinsics = {{{910.0f, 0.0f, 640.0f}, {0.0f, 910.0f, 400.0f}, {0.0f, 0.0f, 1.0f}}};
    const std::vector<float> fromDist = {0.085f, -0.21f, 0.0015f, -0.0008f, 0.11f, 0.004f, -0.002f, 0.001f, 0.0004f, -0.0002f, 0.0001f, -0.0001f, 0.005f, -0.004f};
    const std::vector<float> toDist = {0.11f, -0.24f, 0.0013f, -0.0005f, 0.13f, 0.003f, -0.001f, 0.0015f, -0.0003f, 0.0002f, -0.0001f, 0.0001f, -0.004f, 0.006f};

    dai::ImgTransformation from(1280, 800);
    from.setIntrinsicMatrix(intrinsics);
    from.setDistortionModel(dai::CameraModel::Perspective);
    from.setDistortionCoefficients(fromDist);
    from.setExtrinsics(makeIdentityExtrinsics());

    dai::ImgTransformation to(1280, 800);
    to.setIntrinsicMatrix(intrinsics);
    to.setDistortionModel(dai::CameraModel::Perspective);
    to.setDistortionCoefficients(toDist);
    to.setExtrinsics(makeIdentityExtrinsics());

    const std::vector<dai::Point2f> points = {
        {640.0f, 400.0f},
        {420.0f, 300.0f},
        {980.0f, 300.0f},
        {420.0f, 650.0f},
        {980.0f, 650.0f},
    };

    for(const auto& p : points) {
        const auto noCv = from.remapPointTo(to, p);
        std::vector<cv::Point2f> src = {cv::Point2f(p.x, p.y)};
        std::vector<cv::Point2f> undistorted;
        cv::Mat fromDistCv(1, static_cast<int>(fromDist.size()), CV_32F);
        for(size_t i = 0; i < fromDist.size(); ++i) fromDistCv.at<float>(0, static_cast<int>(i)) = fromDist[i];
        cv::Matx33f kFrom(intrinsics[0][0], intrinsics[0][1], intrinsics[0][2], intrinsics[1][0], intrinsics[1][1], intrinsics[1][2], intrinsics[2][0], intrinsics[2][1], intrinsics[2][2]);
        cv::Matx33f kTo(intrinsics[0][0], intrinsics[0][1], intrinsics[0][2], intrinsics[1][0], intrinsics[1][1], intrinsics[1][2], intrinsics[2][0], intrinsics[2][1], intrinsics[2][2]);
        cv::undistortPoints(src, undistorted, kFrom, fromDistCv, cv::noArray(), cv::noArray());

        std::vector<cv::Point3f> objectPoints = {cv::Point3f(undistorted[0].x, undistorted[0].y, 1.0f)};
        std::vector<cv::Point2f> projected;
        cv::Mat toDistCv(1, static_cast<int>(toDist.size()), CV_32F);
        for(size_t i = 0; i < toDist.size(); ++i) toDistCv.at<float>(0, static_cast<int>(i)) = toDist[i];
        cv::projectPoints(objectPoints, cv::Vec3f(0.0f, 0.0f, 0.0f), cv::Vec3f(0.0f, 0.0f, 0.0f), kTo, toDistCv, projected);
        const auto cvRef = projected[0];
        REQUIRE_THAT(noCv.x, Catch::Matchers::WithinAbs(cvRef.x, 0.05f));
        REQUIRE_THAT(noCv.y, Catch::Matchers::WithinAbs(cvRef.y, 0.05f));
    }
}
#endif
