#include <catch2/catch_all.hpp>

#ifdef DEPTHAI_HAVE_OPENCV_SUPPORT

    #include <opencv2/calib3d.hpp>
    #include <opencv2/imgproc.hpp>
    #include <random>
    #include <vector>

    #include "pipeline/node/MultiDeviceScaleEstimation.hpp"

using namespace dai::impl;

namespace {

cv::Matx33d rotY(double deg) {
    const double r = deg * CV_PI / 180.0;
    return cv::Matx33d(std::cos(r), 0, std::sin(r), 0, 1, 0, -std::sin(r), 0, std::cos(r));
}

cv::Mat makeTexture(int w, int h, unsigned seed) {
    cv::Mat small(h / 6, w / 6, CV_8UC1);
    cv::RNG(seed).fill(small, cv::RNG::UNIFORM, 0, 255);
    cv::Mat big;
    cv::resize(small, big, cv::Size(w, h), 0, 0, cv::INTER_CUBIC);
    cv::GaussianBlur(big, big, cv::Size(0, 0), 1.0);
    return big;
}

struct Cam {
    cv::Matx33d Rcw;
    cv::Vec3d C;
    cv::Matx33d K;
};
struct Plane {
    cv::Vec3d origin, e1, e2;
    cv::Mat texture;
};

cv::Point2d project(const Cam& cam, const cv::Vec3d& Xw, double& depth) {
    const cv::Vec3d Xc = cam.Rcw * (Xw - cam.C);
    depth = Xc(2);
    return cv::Point2d(cam.K(0, 0) * Xc(0) / Xc(2) + cam.K(0, 2), cam.K(1, 1) * Xc(1) / Xc(2) + cam.K(1, 2));
}

cv::Mat renderView(const Cam& cam, const std::vector<Plane>& planes, cv::Size size, unsigned noiseSeed) {
    cv::Mat canvas = cv::Mat::zeros(size, CV_8UC1);
    std::vector<std::pair<double, const Plane*>> order;
    for(const auto& p : planes) {
        double d;
        project(cam, p.origin + 0.5 * p.e1 + 0.5 * p.e2, d);
        order.emplace_back(d, &p);
    }
    std::sort(order.begin(), order.end(), [](auto& a, auto& b) { return a.first > b.first; });
    const cv::Vec3d cornersUV[4] = {{0, 0, 0}, {1, 0, 0}, {1, 1, 0}, {0, 1, 0}};
    for(auto& [d, pp] : order) {
        const Plane& p = *pp;
        std::vector<cv::Point2f> src = {{0, 0}, {(float)p.texture.cols, 0}, {(float)p.texture.cols, (float)p.texture.rows}, {0, (float)p.texture.rows}};
        std::vector<cv::Point2f> dst(4);
        bool inFront = true;
        for(int i = 0; i < 4; ++i) {
            double depth;
            cv::Vec3d Xw = p.origin + cornersUV[i](0) * p.e1 + cornersUV[i](1) * p.e2;
            cv::Point2d px = project(cam, Xw, depth);
            if(depth <= 0.05) inFront = false;
            dst[i] = cv::Point2f((float)px.x, (float)px.y);
        }
        if(!inFront) continue;
        cv::Mat H = cv::getPerspectiveTransform(src, dst);
        cv::Mat warped, mask;
        cv::warpPerspective(p.texture, warped, H, size, cv::INTER_LINEAR, cv::BORDER_CONSTANT, 0);
        cv::Mat ones(p.texture.size(), CV_8UC1, cv::Scalar(255));
        cv::warpPerspective(ones, mask, H, size, cv::INTER_NEAREST, cv::BORDER_CONSTANT, 0);
        warped.copyTo(canvas, mask);
    }
    cv::Mat noise(size, CV_8UC1);
    cv::RNG(noiseSeed).fill(noise, cv::RNG::NORMAL, 0, 3);
    cv::add(canvas, noise, canvas);
    return canvas;
}

}  // namespace

TEST_CASE("kabsch recovers a known rigid transform exactly") {
    std::mt19937 rng(1);
    std::normal_distribution<double> nd(0, 1);
    const cv::Matx33d Rgt = rotY(20.0);
    const cv::Vec3d tgt(0.3, -0.1, 1.2);
    std::vector<cv::Point3d> A, B;
    for(int i = 0; i < 50; ++i) {
        cv::Vec3d b(nd(rng), nd(rng), nd(rng) + 3.0);
        cv::Vec3d a = Rgt * b + tgt;
        B.emplace_back(b(0), b(1), b(2));
        A.emplace_back(a(0), a(1), a(2));
    }
    cv::Matx33d R;
    cv::Vec3d t;
    REQUIRE(kabsch(A, B, R, t));
    REQUIRE(cv::norm(t - tgt) < 1e-9);
    REQUIRE(cv::norm(cv::Mat(R - Rgt)) < 1e-9);
}

TEST_CASE("kabschRansac recovers translation magnitude with noise and outliers") {
    std::mt19937 rng(7);
    std::normal_distribution<double> noise(0, 0.01);
    std::uniform_real_distribution<double> outlier(-2, 2);
    std::uniform_real_distribution<double> u01(0, 1);
    const cv::Matx33d Rgt = rotY(-15.0);
    const cv::Vec3d tgt(1.0, 0.05, 0.2);
    std::vector<cv::Point3d> A, B;
    for(int i = 0; i < 300; ++i) {
        cv::Vec3d b(outlier(rng), outlier(rng), std::abs(outlier(rng)) + 2.0);
        cv::Vec3d a = (u01(rng) < 0.30) ? cv::Vec3d(outlier(rng), outlier(rng), outlier(rng) + 2.0)
                                        : cv::Vec3d(Rgt * b + tgt + cv::Vec3d(noise(rng), noise(rng), noise(rng)));
        B.emplace_back(b(0), b(1), b(2));
        A.emplace_back(a(0), a(1), a(2));
    }
    RigidFit fit = kabschRansac(A, B, 0.05, 3000, 0);
    REQUIRE(fit.ok);
    REQUIRE(std::abs(cv::norm(fit.t) - cv::norm(tgt)) < 0.03);
}

TEST_CASE("triangulateMetric recovers known metric depth") {
    const cv::Matx33d K(500, 0, 320, 0, 500, 240, 0, 0, 1);
    const double baseline = 0.075;
    cv::Matx44d leftFromRight = cv::Matx44d::eye();
    leftFromRight(0, 3) = baseline;
    std::vector<cv::Point3d> gt;
    std::vector<cv::Point2d> lpx, rpx;
    std::mt19937 rng(3);
    std::uniform_real_distribution<double> ud(-0.8, 0.8), zd(1.5, 4.0);
    for(int i = 0; i < 40; ++i) {
        cv::Point3d Xl(ud(rng), ud(rng), zd(rng));
        lpx.emplace_back(K(0, 0) * Xl.x / Xl.z + K(0, 2), K(1, 1) * Xl.y / Xl.z + K(1, 2));
        rpx.emplace_back(K(0, 0) * (Xl.x - baseline) / Xl.z + K(0, 2), K(1, 1) * Xl.y / Xl.z + K(1, 2));
        gt.push_back(Xl);
    }
    std::vector<cv::Point3d> pts;
    std::vector<char> keep;
    triangulateMetric(lpx, rpx, K, K, leftFromRight, 1.0, pts, keep);
    double maxErr = 0;
    int kept = 0;
    for(size_t i = 0; i < gt.size(); ++i) {
        if(!keep[i]) continue;
        ++kept;
        maxErr = std::max(maxErr, cv::norm(pts[i] - gt[i]));
    }
    REQUIRE(kept == (int)gt.size());
    REQUIRE(maxErr < 1e-4);
}

TEST_CASE("estimateInterDeviceScale recovers the inter-device distance from a rendered scene") {
    const cv::Matx33d K(500, 0, 320, 0, 500, 240, 0, 0, 1);
    const cv::Size sz(640, 480);
    const double b = 0.075;
    std::vector<Plane> planes = {
        {{-1.5, -1.0, 3.0}, {3.0, 0, 0}, {0, 2.0, 0}, makeTexture(512, 384, 11)},
        {{-1.5, -1.0, 3.0}, {0, 0, -2.0}, {0, 2.0, 0}, makeTexture(384, 384, 22)},
        {{-1.5, 1.0, 3.0}, {3.0, 0, 0}, {0, 0, -2.0}, makeTexture(512, 384, 33)},
    };
    const cv::Vec3d centerBLeft(0.9, 0.0, 0.3);
    const double gtDistance = cv::norm(centerBLeft);
    const cv::Matx33d Rwc_A = cv::Matx33d::eye();
    const cv::Matx33d Rwc_B = rotY(-22.0);
    auto makeCam = [&](const cv::Matx33d& Rwc, const cv::Vec3d& center) {
        Cam c;
        c.Rcw = Rwc.t();
        c.C = center;
        c.K = K;
        return c;
    };
    Cam aLeft = makeCam(Rwc_A, cv::Vec3d(0, 0, 0));
    Cam aRight = makeCam(Rwc_A, Rwc_A * cv::Vec3d(b, 0, 0));
    Cam bLeft = makeCam(Rwc_B, centerBLeft);
    Cam bRight = makeCam(Rwc_B, centerBLeft + Rwc_B * cv::Vec3d(b, 0, 0));

    StereoDeviceViews A, B;
    A.intrinsicsLeft = A.intrinsicsRight = K;
    B.intrinsicsLeft = B.intrinsicsRight = K;
    A.leftFromRight = cv::Matx44d::eye();
    A.leftFromRight(0, 3) = b;
    B.leftFromRight = cv::Matx44d::eye();
    B.leftFromRight(0, 3) = b;
    for(unsigned f = 0; f < 3; ++f) {
        A.left.push_back(renderView(aLeft, planes, sz, 100 + f));
        A.right.push_back(renderView(aRight, planes, sz, 200 + f));
        B.left.push_back(renderView(bLeft, planes, sz, 300 + f));
        B.right.push_back(renderView(bRight, planes, sz, 400 + f));
    }

    InterDeviceScaleResult r = estimateInterDeviceScale(A, B, ScaleParams{});
    INFO("dist=" << r.distanceMeters << " gt=" << gtDistance << " inliers=" << r.inliers << " rmse=" << r.rmseMeters << " note=" << r.note);
    REQUIRE(r.observable);
    REQUIRE(std::abs(r.distanceMeters - gtDistance) < 0.10 * gtDistance);
}

#else

TEST_CASE("multi-device scale estimation requires OpenCV") {
    SUCCEED("built without OpenCV support");
}

#endif
