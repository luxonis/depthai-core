#include <catch2/catch_all.hpp>

#ifdef DEPTHAI_HAVE_OPENCV_SUPPORT

    #include <opencv2/calib3d.hpp>
    #include <opencv2/imgproc.hpp>
    #include <random>

    #include "pipeline/node/MultiDeviceScaleEstimation.hpp"

using namespace dai::impl;

namespace {

cv::Matx33d rotY(double degrees) {
    const double radians = degrees * CV_PI / 180.0;
    return cv::Matx33d(std::cos(radians), 0, std::sin(radians), 0, 1, 0, -std::sin(radians), 0, std::cos(radians));
}

cv::Mat makeTexture(int width, int height, unsigned seed) {
    cv::Mat small(height / 6, width / 6, CV_8UC1);
    cv::RNG(seed).fill(small, cv::RNG::UNIFORM, 0, 255);
    cv::Mat large;
    cv::resize(small, large, cv::Size(width, height), 0, 0, cv::INTER_CUBIC);
    cv::GaussianBlur(large, large, cv::Size(0, 0), 1.0);
    return large;
}

struct Camera {
    cv::Matx33d rotationCameraFromWorld;
    cv::Vec3d center;
    cv::Matx33d intrinsics;
};

struct Plane {
    cv::Vec3d origin;
    cv::Vec3d axis1;
    cv::Vec3d axis2;
    cv::Mat texture;
};

cv::Point2d project(const Camera& camera, const cv::Vec3d& pointWorld, double& depth) {
    const cv::Vec3d pointCamera = camera.rotationCameraFromWorld * (pointWorld - camera.center);
    depth = pointCamera[2];
    return {camera.intrinsics(0, 0) * pointCamera[0] / pointCamera[2] + camera.intrinsics(0, 2),
            camera.intrinsics(1, 1) * pointCamera[1] / pointCamera[2] + camera.intrinsics(1, 2)};
}

cv::Mat renderView(const Camera& camera, const std::vector<Plane>& planes, cv::Size size, unsigned noiseSeed) {
    cv::Mat canvas = cv::Mat::zeros(size, CV_8UC1);
    std::vector<std::pair<double, const Plane*>> order;
    for(const auto& plane : planes) {
        double depth;
        project(camera, plane.origin + 0.5 * plane.axis1 + 0.5 * plane.axis2, depth);
        order.emplace_back(depth, &plane);
    }
    std::sort(order.begin(), order.end(), [](const auto& left, const auto& right) { return left.first > right.first; });

    for(const auto& [unused, plane] : order) {
        (void)unused;
        const cv::Vec3d corners[4] = {plane->origin, plane->origin + plane->axis1, plane->origin + plane->axis1 + plane->axis2, plane->origin + plane->axis2};
        std::vector<cv::Point2f> source = {{0, 0},
                                           {static_cast<float>(plane->texture.cols), 0},
                                           {static_cast<float>(plane->texture.cols), static_cast<float>(plane->texture.rows)},
                                           {0, static_cast<float>(plane->texture.rows)}};
        std::vector<cv::Point2f> destination(4);
        bool inFront = true;
        for(int i = 0; i < 4; ++i) {
            double depth;
            const auto pixel = project(camera, corners[i], depth);
            inFront = inFront && depth > 0.05;
            destination[i] = cv::Point2f(static_cast<float>(pixel.x), static_cast<float>(pixel.y));
        }
        if(!inFront) continue;

        const auto homography = cv::getPerspectiveTransform(source, destination);
        cv::Mat warped, mask;
        cv::warpPerspective(plane->texture, warped, homography, size, cv::INTER_LINEAR, cv::BORDER_CONSTANT, 0);
        cv::Mat ones(plane->texture.size(), CV_8UC1, cv::Scalar(255));
        cv::warpPerspective(ones, mask, homography, size, cv::INTER_NEAREST, cv::BORDER_CONSTANT, 0);
        warped.copyTo(canvas, mask);
    }

    cv::Mat noise(size, CV_8UC1);
    cv::RNG(noiseSeed).fill(noise, cv::RNG::NORMAL, 0, 3);
    cv::add(canvas, noise, canvas);
    return canvas;
}

}  // namespace

TEST_CASE("multi-device scale Kabsch recovers a rigid transform") {
    const cv::Matx33d expectedRotation = rotY(20.0);
    const cv::Vec3d expectedTranslation(0.3, -0.1, 1.2);
    std::mt19937 generator(1);
    std::normal_distribution<double> normal(0, 1);
    std::vector<cv::Point3d> pointsA, pointsB;
    for(int i = 0; i < 50; ++i) {
        const cv::Vec3d pointB(normal(generator), normal(generator), normal(generator) + 3.0);
        const auto pointA = expectedRotation * pointB + expectedTranslation;
        pointsB.emplace_back(pointB[0], pointB[1], pointB[2]);
        pointsA.emplace_back(pointA[0], pointA[1], pointA[2]);
    }

    cv::Matx33d rotation;
    cv::Vec3d translation;
    REQUIRE(kabsch(pointsA, pointsB, rotation, translation));
    REQUIRE(cv::norm(translation - expectedTranslation) < 1e-9);
    REQUIRE(cv::norm(cv::Mat(rotation - expectedRotation)) < 1e-9);
}

TEST_CASE("multi-device scale recovers distance from a rendered four-view scene") {
    const cv::Matx33d intrinsics(500, 0, 320, 0, 500, 240, 0, 0, 1);
    const cv::Size imageSize(640, 480);
    const double stereoBaseline = 0.075;
    const std::vector<Plane> planes = {
        {{-1.5, -1.0, 3.0}, {3.0, 0, 0}, {0, 2.0, 0}, makeTexture(512, 384, 11)},
        {{-1.5, -1.0, 3.0}, {0, 0, -2.0}, {0, 2.0, 0}, makeTexture(384, 384, 22)},
        {{-1.5, 1.0, 3.0}, {3.0, 0, 0}, {0, 0, -2.0}, makeTexture(512, 384, 33)},
    };
    const cv::Vec3d deviceBCenter(0.9, 0.0, 0.3);
    const double expectedDistance = cv::norm(deviceBCenter);
    const cv::Matx33d worldFromA = cv::Matx33d::eye();
    const cv::Matx33d worldFromB = rotY(-22.0);
    const auto makeCamera = [&](const cv::Matx33d& worldFromCamera, const cv::Vec3d& center) { return Camera{worldFromCamera.t(), center, intrinsics}; };

    const Camera aLeft = makeCamera(worldFromA, {0, 0, 0});
    const Camera aRight = makeCamera(worldFromA, worldFromA * cv::Vec3d(stereoBaseline, 0, 0));
    const Camera bLeft = makeCamera(worldFromB, deviceBCenter);
    const Camera bRight = makeCamera(worldFromB, deviceBCenter + worldFromB * cv::Vec3d(stereoBaseline, 0, 0));

    StereoDeviceViews deviceA, deviceB;
    deviceA.intrinsicsLeft = deviceA.intrinsicsRight = intrinsics;
    deviceB.intrinsicsLeft = deviceB.intrinsicsRight = intrinsics;
    deviceA.leftFromRight = deviceB.leftFromRight = cv::Matx44d::eye();
    deviceA.leftFromRight(0, 3) = stereoBaseline;
    deviceB.leftFromRight(0, 3) = stereoBaseline;
    for(unsigned frame = 0; frame < 3; ++frame) {
        deviceA.left.push_back(renderView(aLeft, planes, imageSize, 100 + frame));
        deviceA.right.push_back(renderView(aRight, planes, imageSize, 200 + frame));
        deviceB.left.push_back(renderView(bLeft, planes, imageSize, 300 + frame));
        deviceB.right.push_back(renderView(bRight, planes, imageSize, 400 + frame));
    }

    const auto result = estimateInterDeviceScale(deviceA, deviceB);
    INFO("distance=" << result.distanceMeters << " expected=" << expectedDistance << " inliers=" << result.inliers << " rmse=" << result.rmseMeters
                     << " note=" << result.note);
    REQUIRE(result.observable);
    REQUIRE(std::abs(result.distanceMeters - expectedDistance) < 0.10 * expectedDistance);
}

#else

TEST_CASE("multi-device scale estimation without OpenCV") {
    SUCCEED("built without OpenCV support");
}

#endif
