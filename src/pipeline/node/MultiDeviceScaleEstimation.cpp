#include "pipeline/node/MultiDeviceScaleEstimation.hpp"

#ifdef DEPTHAI_HAVE_OPENCV_SUPPORT

    #include <algorithm>
    #include <cmath>
    #include <limits>
    #include <map>
    #include <numeric>
    #include <opencv2/calib3d.hpp>
    #include <opencv2/features2d.hpp>
    #include <opencv2/flann.hpp>
    #include <opencv2/imgproc.hpp>
    #include <random>
    #include <utility>

namespace dai {
namespace impl {

namespace {

cv::Mat toGray(const cv::Mat& image) {
    if(image.empty()) return image;
    if(image.channels() == 1) return image;
    cv::Mat gray;
    cv::cvtColor(image, gray, cv::COLOR_BGR2GRAY);
    return gray;
}

cv::Mat intrinsicsToMat(const cv::Matx33d& k) {
    return cv::Mat(k);
}

/// Warp `image` by a simulated tilt/rotation and return the warped image plus the inverse affine that maps
/// warped-image coordinates back to the original image. Mirrors the ASIFT affine simulation.
cv::Mat affineSkew(double tilt, double phi, const cv::Mat& image, cv::Matx23d& invAffine) {
    const int h = image.rows;
    const int w = image.cols;
    cv::Matx23d affine(1, 0, 0, 0, 1, 0);
    cv::Mat working = image;

    if(phi != 0.0) {
        const double rad = phi * CV_PI / 180.0;
        const double s = std::sin(rad);
        const double c = std::cos(rad);
        affine = cv::Matx23d(c, -s, 0, s, c, 0);
        std::vector<cv::Point2f> corners = {{0, 0}, {float(w), 0}, {float(w), float(h)}, {0, float(h)}};
        std::vector<cv::Point2f> transformed(corners.size());
        for(size_t i = 0; i < corners.size(); ++i) {
            transformed[i] = cv::Point2f(float(affine(0, 0) * corners[i].x + affine(0, 1) * corners[i].y),
                                         float(affine(1, 0) * corners[i].x + affine(1, 1) * corners[i].y));
        }
        const cv::Rect bounds = cv::boundingRect(transformed);
        affine(0, 2) = -bounds.x;
        affine(1, 2) = -bounds.y;
        cv::warpAffine(working, working, cv::Mat(affine), cv::Size(bounds.width, bounds.height), cv::INTER_LINEAR, cv::BORDER_REPLICATE);
    }
    if(tilt != 1.0) {
        const double sigma = 0.8 * std::sqrt(tilt * tilt - 1.0);
        cv::GaussianBlur(working, working, cv::Size(0, 0), sigma, 0.01);
        cv::resize(working, working, cv::Size(0, 0), 1.0 / tilt, 1.0, cv::INTER_NEAREST);
        affine(0, 0) /= tilt;
        affine(0, 1) /= tilt;
        affine(0, 2) /= tilt;
    }
    cv::invertAffineTransform(cv::Mat(affine), invAffine);
    return working;
}

/// ASIFT (or plain SIFT) keypoints deduplicated by location, keeping the strongest response per bucket.
void asiftDetect(const cv::Mat& gray, bool useAsift, double dedupPx, std::vector<cv::Point2d>& points, cv::Mat& descriptors) {
    cv::Ptr<cv::SIFT> detector = cv::SIFT::create(0, 3, 0.03, 12);

    std::vector<std::pair<double, double>> params = {{1.0, 0.0}};
    if(useAsift) {
        for(int i = 1; i < 4; ++i) {
            const double tilt = std::pow(2.0, 0.5 * i);
            for(double phi = 0.0; phi < 180.0; phi += 72.0 / tilt) {
                params.emplace_back(tilt, phi);
            }
        }
    }

    std::vector<cv::Point2d> allPts;
    std::vector<float> allResp;
    cv::Mat allDesc;
    for(const auto& [tilt, phi] : params) {
        cv::Matx23d invAffine;
        cv::Mat warped = affineSkew(tilt, phi, gray, invAffine);
        std::vector<cv::KeyPoint> keypoints;
        cv::Mat desc;
        detector->detectAndCompute(warped, cv::noArray(), keypoints, desc);
        if(desc.empty()) continue;
        for(size_t i = 0; i < keypoints.size(); ++i) {
            const double x = keypoints[i].pt.x;
            const double y = keypoints[i].pt.y;
            const double ox = invAffine(0, 0) * x + invAffine(0, 1) * y + invAffine(0, 2);
            const double oy = invAffine(1, 0) * x + invAffine(1, 1) * y + invAffine(1, 2);
            allPts.emplace_back(ox, oy);
            allResp.push_back(keypoints[i].response);
            allDesc.push_back(desc.row(static_cast<int>(i)));
        }
    }

    points.clear();
    descriptors.release();
    if(allPts.empty()) return;

    std::vector<int> order(allPts.size());
    std::iota(order.begin(), order.end(), 0);
    std::sort(order.begin(), order.end(), [&allResp](int a, int b) { return allResp[a] > allResp[b]; });

    std::map<std::pair<long long, long long>, bool> seen;
    std::vector<int> keep;
    for(const int idx : order) {
        const std::pair<long long, long long> key{llround(allPts[idx].x / dedupPx), llround(allPts[idx].y / dedupPx)};
        if(seen.count(key)) continue;
        seen[key] = true;
        keep.push_back(idx);
    }
    std::sort(keep.begin(), keep.end());

    descriptors.create(static_cast<int>(keep.size()), allDesc.cols, allDesc.type());
    for(size_t i = 0; i < keep.size(); ++i) {
        points.push_back(allPts[keep[i]]);
        allDesc.row(keep[i]).copyTo(descriptors.row(static_cast<int>(i)));
    }
}

/// Copy a subset of descriptor rows.
cv::Mat selectRows(const cv::Mat& descriptors, const std::vector<int>& indices) {
    cv::Mat out(static_cast<int>(indices.size()), descriptors.cols, descriptors.type());
    for(size_t i = 0; i < indices.size(); ++i) {
        descriptors.row(indices[i]).copyTo(out.row(static_cast<int>(i)));
    }
    return out;
}

/// FLANN ratio-test matcher with a forward/reverse cross-check. Returns matched (queryIdx, trainIdx) pairs.
std::vector<std::pair<int, int>> crossMatch(const cv::Mat& descA, const cv::Mat& descB, double ratio) {
    std::vector<std::pair<int, int>> result;
    if(descA.rows < 2 || descB.rows < 2) return result;

    cv::FlannBasedMatcher matcher(cv::makePtr<cv::flann::KDTreeIndexParams>(4), cv::makePtr<cv::flann::SearchParams>(32));

    auto goodMatches = [ratio](const std::vector<std::vector<cv::DMatch>>& knn) {
        std::map<int, std::pair<int, float>> good;  // queryIdx -> (trainIdx, distance)
        for(const auto& pair : knn) {
            if(pair.size() < 2) continue;
            if(pair[0].distance < ratio * pair[1].distance) {
                auto it = good.find(pair[0].queryIdx);
                if(it == good.end() || pair[0].distance < it->second.second) {
                    good[pair[0].queryIdx] = {pair[0].trainIdx, pair[0].distance};
                }
            }
        }
        return good;
    };

    std::vector<std::vector<cv::DMatch>> forwardKnn;
    std::vector<std::vector<cv::DMatch>> reverseKnn;
    matcher.knnMatch(descA, descB, forwardKnn, 2);
    matcher.knnMatch(descB, descA, reverseKnn, 2);
    const auto forward = goodMatches(forwardKnn);
    const auto reverse = goodMatches(reverseKnn);

    for(const auto& [query, trainDist] : forward) {
        const auto it = reverse.find(trainDist.first);
        if(it != reverse.end() && it->second.first == query) {
            result.emplace_back(query, trainDist.first);
        }
    }
    return result;
}

std::vector<cv::Point2d> normalizePoints(const std::vector<cv::Point2d>& pixels, const cv::Matx33d& k) {
    if(pixels.empty()) return {};
    std::vector<cv::Point2d> normalized;
    cv::undistortPoints(pixels, normalized, intrinsicsToMat(k), cv::noArray());
    return normalized;
}

std::vector<cv::Point2d> projectPoints(const std::vector<cv::Point3d>& pointsCam, const cv::Matx33d& k) {
    std::vector<cv::Point2d> projected(pointsCam.size(), cv::Point2d(std::nan(""), std::nan("")));
    std::vector<cv::Point3d> valid;
    std::vector<int> validIdx;
    for(size_t i = 0; i < pointsCam.size(); ++i) {
        const auto& p = pointsCam[i];
        if(std::isfinite(p.x) && std::isfinite(p.y) && std::isfinite(p.z) && p.z > 1e-8) {
            valid.push_back(p);
            validIdx.push_back(static_cast<int>(i));
        }
    }
    if(valid.empty()) return projected;
    std::vector<cv::Point2d> out;
    cv::projectPoints(valid, cv::Vec3d(0, 0, 0), cv::Vec3d(0, 0, 0), intrinsicsToMat(k), cv::noArray(), out);
    for(size_t i = 0; i < validIdx.size(); ++i) {
        projected[validIdx[i]] = out[i];
    }
    return projected;
}

cv::Matx44d invertSe3(const cv::Matx44d& transform) {
    cv::Matx33d r;
    cv::Vec3d t;
    for(int i = 0; i < 3; ++i) {
        for(int j = 0; j < 3; ++j) r(i, j) = transform(i, j);
        t(i) = transform(i, 3);
    }
    const cv::Matx33d rt = r.t();
    const cv::Vec3d tInv = -(rt * t);
    cv::Matx44d out = cv::Matx44d::eye();
    for(int i = 0; i < 3; ++i) {
        for(int j = 0; j < 3; ++j) out(i, j) = rt(i, j);
        out(i, 3) = tInv(i);
    }
    return out;
}

}  // namespace

void triangulateMetric(const std::vector<cv::Point2d>& leftPx,
                       const std::vector<cv::Point2d>& rightPx,
                       const cv::Matx33d& intrinsicsLeft,
                       const cv::Matx33d& intrinsicsRight,
                       const cv::Matx44d& leftFromRight,
                       double gatePx,
                       std::vector<cv::Point3d>& pointsLeft,
                       std::vector<char>& keep) {
    pointsLeft.assign(leftPx.size(), cv::Point3d(std::nan(""), std::nan(""), std::nan("")));
    keep.assign(leftPx.size(), 0);
    if(leftPx.empty()) return;

    const std::vector<cv::Point2d> leftNorm = normalizePoints(leftPx, intrinsicsLeft);
    const std::vector<cv::Point2d> rightNorm = normalizePoints(rightPx, intrinsicsRight);

    const cv::Matx44d rightFromLeft = invertSe3(leftFromRight);
    cv::Matx34d projLeft(1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0);
    cv::Matx34d projRight;
    for(int i = 0; i < 3; ++i) {
        for(int j = 0; j < 4; ++j) projRight(i, j) = rightFromLeft(i, j);
    }

    cv::Mat leftMat(2, static_cast<int>(leftNorm.size()), CV_64F);
    cv::Mat rightMat(2, static_cast<int>(rightNorm.size()), CV_64F);
    for(size_t i = 0; i < leftNorm.size(); ++i) {
        leftMat.at<double>(0, static_cast<int>(i)) = leftNorm[i].x;
        leftMat.at<double>(1, static_cast<int>(i)) = leftNorm[i].y;
        rightMat.at<double>(0, static_cast<int>(i)) = rightNorm[i].x;
        rightMat.at<double>(1, static_cast<int>(i)) = rightNorm[i].y;
    }

    cv::Mat points4d;
    cv::triangulatePoints(cv::Mat(projLeft), cv::Mat(projRight), leftMat, rightMat, points4d);

    std::vector<cv::Point3d> pointsRight(leftPx.size(), cv::Point3d(std::nan(""), std::nan(""), std::nan("")));
    for(int i = 0; i < points4d.cols; ++i) {
        const double w = points4d.at<double>(3, i);
        if(std::abs(w) <= 1e-8) continue;
        const cv::Point3d pl(points4d.at<double>(0, i) / w, points4d.at<double>(1, i) / w, points4d.at<double>(2, i) / w);
        pointsLeft[i] = pl;
        const cv::Vec3d pr = cv::Matx33d(rightFromLeft.get_minor<3, 3>(0, 0)) * cv::Vec3d(pl.x, pl.y, pl.z)
                             + cv::Vec3d(rightFromLeft(0, 3), rightFromLeft(1, 3), rightFromLeft(2, 3));
        pointsRight[i] = cv::Point3d(pr(0), pr(1), pr(2));
    }

    const std::vector<cv::Point2d> reprojLeft = projectPoints(pointsLeft, intrinsicsLeft);
    const std::vector<cv::Point2d> reprojRight = projectPoints(pointsRight, intrinsicsRight);
    for(size_t i = 0; i < leftPx.size(); ++i) {
        const cv::Point3d& pl = pointsLeft[i];
        const cv::Point3d& pr = pointsRight[i];
        const bool posDepth = std::isfinite(pl.z) && pl.z > 0.0 && std::isfinite(pr.z) && pr.z > 0.0;
        if(!posDepth) continue;
        const double eLeft = cv::norm(reprojLeft[i] - leftPx[i]);
        const double eRight = cv::norm(reprojRight[i] - rightPx[i]);
        const double err = std::max(eLeft, eRight);
        if(std::isfinite(err) && err <= gatePx) keep[i] = 1;
    }
}

bool kabsch(const std::vector<cv::Point3d>& pointsA, const std::vector<cv::Point3d>& pointsB, cv::Matx33d& R, cv::Vec3d& t) {
    if(pointsA.size() < 3 || pointsA.size() != pointsB.size()) return false;
    cv::Vec3d centroidA(0, 0, 0);
    cv::Vec3d centroidB(0, 0, 0);
    for(size_t i = 0; i < pointsA.size(); ++i) {
        centroidA += cv::Vec3d(pointsA[i].x, pointsA[i].y, pointsA[i].z);
        centroidB += cv::Vec3d(pointsB[i].x, pointsB[i].y, pointsB[i].z);
    }
    centroidA *= 1.0 / pointsA.size();
    centroidB *= 1.0 / pointsB.size();

    cv::Matx33d h = cv::Matx33d::zeros();  // H = sum (B-cb)(A-ca)^T
    for(size_t i = 0; i < pointsA.size(); ++i) {
        const cv::Vec3d a = cv::Vec3d(pointsA[i].x, pointsA[i].y, pointsA[i].z) - centroidA;
        const cv::Vec3d b = cv::Vec3d(pointsB[i].x, pointsB[i].y, pointsB[i].z) - centroidB;
        h += b * a.t();
    }

    cv::Matx33d u, vt;
    cv::Matx31d w;
    cv::SVD::compute(h, w, u, vt);
    cv::Matx33d v = vt.t();
    const double d = cv::determinant(v * u.t()) < 0 ? -1.0 : 1.0;
    const cv::Matx33d diag(1, 0, 0, 0, 1, 0, 0, 0, d);
    R = v * diag * u.t();
    t = centroidA - R * centroidB;
    return true;
}

RigidFit kabschRansac(const std::vector<cv::Point3d>& pointsA,
                      const std::vector<cv::Point3d>& pointsB,
                      double threshold,
                      int iterations,
                      unsigned seed) {
    RigidFit fit;
    const int n = static_cast<int>(pointsA.size());
    if(n < 3 || pointsA.size() != pointsB.size()) return fit;

    std::mt19937 rng(seed);
    std::uniform_int_distribution<int> pick(0, n - 1);

    auto residuals = [&](const cv::Matx33d& R, const cv::Vec3d& t) {
        std::vector<double> res(n);
        for(int i = 0; i < n; ++i) {
            const cv::Vec3d b(pointsB[i].x, pointsB[i].y, pointsB[i].z);
            const cv::Vec3d a(pointsA[i].x, pointsA[i].y, pointsA[i].z);
            res[i] = cv::norm(a - (R * b + t));
        }
        return res;
    };

    std::vector<char> bestInliers;
    int bestCount = 0;
    for(int iter = 0; iter < iterations; ++iter) {
        int i0 = pick(rng), i1 = pick(rng), i2 = pick(rng);
        if(i0 == i1 || i1 == i2 || i0 == i2) continue;
        std::vector<cv::Point3d> sampleA = {pointsA[i0], pointsA[i1], pointsA[i2]};
        std::vector<cv::Point3d> sampleB = {pointsB[i0], pointsB[i1], pointsB[i2]};
        cv::Matx33d R;
        cv::Vec3d t;
        if(!kabsch(sampleA, sampleB, R, t)) continue;
        const std::vector<double> res = residuals(R, t);
        std::vector<char> inliers(n);
        int count = 0;
        for(int i = 0; i < n; ++i) {
            inliers[i] = res[i] < threshold ? 1 : 0;
            count += inliers[i];
        }
        if(count > bestCount) {
            bestCount = count;
            bestInliers = inliers;
        }
    }
    if(bestInliers.empty() || bestCount < 3) return fit;

    // Refit on the inlier set, then run a few Huber IRLS iterations for robustness.
    std::vector<cv::Point3d> inA, inB;
    for(int i = 0; i < n; ++i) {
        if(bestInliers[i]) {
            inA.push_back(pointsA[i]);
            inB.push_back(pointsB[i]);
        }
    }
    cv::Matx33d R;
    cv::Vec3d t;
    if(!kabsch(inA, inB, R, t)) return fit;

    for(int iter = 0; iter < 10 && inA.size() >= 3; ++iter) {
        std::vector<double> res(inA.size());
        for(size_t i = 0; i < inA.size(); ++i) {
            const cv::Vec3d b(inB[i].x, inB[i].y, inB[i].z);
            const cv::Vec3d a(inA[i].x, inA[i].y, inA[i].z);
            res[i] = cv::norm(a - (R * b + t));
        }
        std::vector<double> sorted = res;
        std::nth_element(sorted.begin(), sorted.begin() + sorted.size() / 2, sorted.end());
        const double medianRes = sorted[sorted.size() / 2];
        const double huber = std::max(1.345 * medianRes, 1e-3);

        std::vector<double> weights(inA.size());
        double weightSum = 0.0;
        for(size_t i = 0; i < inA.size(); ++i) {
            weights[i] = res[i] > huber ? huber / res[i] : 1.0;
            weightSum += weights[i];
        }
        cv::Vec3d ca(0, 0, 0), cb(0, 0, 0);
        for(size_t i = 0; i < inA.size(); ++i) {
            const double w = weights[i] / weightSum;
            ca += w * cv::Vec3d(inA[i].x, inA[i].y, inA[i].z);
            cb += w * cv::Vec3d(inB[i].x, inB[i].y, inB[i].z);
        }
        cv::Matx33d h = cv::Matx33d::zeros();
        for(size_t i = 0; i < inA.size(); ++i) {
            const cv::Vec3d a = cv::Vec3d(inA[i].x, inA[i].y, inA[i].z) - ca;
            const cv::Vec3d b = cv::Vec3d(inB[i].x, inB[i].y, inB[i].z) - cb;
            h += weights[i] * (b * a.t());
        }
        cv::Matx33d u, vt;
        cv::Matx31d wsv;
        cv::SVD::compute(h, wsv, u, vt);
        cv::Matx33d v = vt.t();
        const double det = cv::determinant(v * u.t()) < 0 ? -1.0 : 1.0;
        const cv::Matx33d diag(1, 0, 0, 0, 1, 0, 0, 0, det);
        R = v * diag * u.t();
        t = ca - R * cb;
    }

    double sumSq = 0.0;
    for(size_t i = 0; i < inA.size(); ++i) {
        const cv::Vec3d b(inB[i].x, inB[i].y, inB[i].z);
        const cv::Vec3d a(inA[i].x, inA[i].y, inA[i].z);
        sumSq += std::pow(cv::norm(a - (R * b + t)), 2.0);
    }
    fit.R = R;
    fit.t = t;
    fit.inliers = bestInliers;
    fit.rmse = inA.empty() ? std::numeric_limits<double>::infinity() : std::sqrt(sumSq / inA.size());
    fit.ok = true;
    return fit;
}

namespace {

/// Undistort every per-frame image to a pinhole model with the same intrinsics.
std::vector<cv::Mat> undistortAll(const std::vector<cv::Mat>& images, const cv::Matx33d& k, const std::vector<double>& dist) {
    std::vector<cv::Mat> out;
    out.reserve(images.size());
    const cv::Mat distMat = dist.empty() ? cv::Mat() : cv::Mat(dist).clone();
    for(const auto& image : images) {
        cv::Mat undistorted;
        cv::undistort(toGray(image), undistorted, intrinsicsToMat(k), distMat);
        out.push_back(undistorted);
    }
    return out;
}

}  // namespace

InterDeviceScaleResult estimateInterDeviceScale(const StereoDeviceViews& deviceA, const StereoDeviceViews& deviceB, const ScaleParams& params) {
    InterDeviceScaleResult result;

    const size_t frames = std::min({deviceA.left.size(), deviceA.right.size(), deviceB.left.size(), deviceB.right.size()});
    if(frames == 0) {
        result.note = "no synchronized frames to estimate the inter-device scale from";
        return result;
    }

    const std::vector<cv::Mat> aLeft = undistortAll(deviceA.left, deviceA.intrinsicsLeft, deviceA.distortionLeft);
    const std::vector<cv::Mat> aRight = undistortAll(deviceA.right, deviceA.intrinsicsRight, deviceA.distortionRight);
    const std::vector<cv::Mat> bLeft = undistortAll(deviceB.left, deviceB.intrinsicsLeft, deviceB.distortionLeft);
    const std::vector<cv::Mat> bRight = undistortAll(deviceB.right, deviceB.intrinsicsRight, deviceB.distortionRight);

    std::vector<cv::Point3d> metricA;  // four-view metric points in device A's left frame
    std::vector<cv::Point3d> metricB;  // matching metric points in device B's left frame

    for(size_t fi = 0; fi < frames; ++fi) {
        std::vector<cv::Point2d> p1, p2, p3, p4;
        cv::Mat d1, d2, d3, d4;
        asiftDetect(aLeft[fi], params.useAsift, params.dedupPx, p1, d1);
        asiftDetect(aRight[fi], params.useAsift, params.dedupPx, p2, d2);
        asiftDetect(bLeft[fi], params.useAsift, params.dedupPx, p3, d3);
        asiftDetect(bRight[fi], params.useAsift, params.dedupPx, p4, d4);

        const auto matchesA = crossMatch(d1, d2, params.ratio);
        const auto matchesB = crossMatch(d3, d4, params.ratio);
        if(matchesA.empty() || matchesB.empty()) continue;

        std::vector<cv::Point2d> a1, a2, b3, b4;
        for(const auto& [qi, ti] : matchesA) {
            a1.push_back(p1[qi]);
            a2.push_back(p2[ti]);
        }
        for(const auto& [qi, ti] : matchesB) {
            b3.push_back(p3[qi]);
            b4.push_back(p4[ti]);
        }

        std::vector<cv::Point3d> xA, xB;
        std::vector<char> keepA, keepB;
        triangulateMetric(a1, a2, deviceA.intrinsicsLeft, deviceA.intrinsicsRight, deviceA.leftFromRight, params.stereoGatePx, xA, keepA);
        triangulateMetric(b3, b4, deviceB.intrinsicsLeft, deviceB.intrinsicsRight, deviceB.leftFromRight, params.stereoGatePx, xB, keepB);

        // Keep only well-triangulated stereo points within a trustworthy depth range, remembering their keypoint
        // indices in each camera so their descriptors can be cross-matched across devices.
        std::vector<int> aKp1, aKp2;
        std::vector<cv::Point3d> aPts;
        for(size_t i = 0; i < matchesA.size(); ++i) {
            if(!keepA[i] || xA[i].z < params.depthMin || xA[i].z > params.depthMax) continue;
            aKp1.push_back(matchesA[i].first);
            aKp2.push_back(matchesA[i].second);
            aPts.push_back(xA[i]);
        }
        std::vector<int> bKp3, bKp4;
        std::vector<cv::Point3d> bPts;
        for(size_t i = 0; i < matchesB.size(); ++i) {
            if(!keepB[i] || xB[i].z < params.depthMin || xB[i].z > params.depthMax) continue;
            bKp3.push_back(matchesB[i].first);
            bKp4.push_back(matchesB[i].second);
            bPts.push_back(xB[i]);
        }
        if(aPts.size() < 4 || bPts.size() < 4) continue;

        // Associate the two devices' metric stereo points by descriptor over both the CAM_B and CAM_C links, with a
        // fundamental-matrix consistency filter, and record the resulting metric 3D-3D correspondences.
        auto associate = [&](const cv::Mat& descAll_A, const std::vector<int>& kpA, const std::vector<cv::Point2d>& ptsA,
                             const cv::Mat& descAll_B, const std::vector<int>& kpB, const std::vector<cv::Point2d>& ptsB) {
            const cv::Mat descA = selectRows(descAll_A, kpA);
            const cv::Mat descB = selectRows(descAll_B, kpB);
            auto matches = crossMatch(descA, descB, params.ratio);
            if(matches.empty()) return;
            std::vector<cv::Point2d> pa, pb;
            for(const auto& [qi, ti] : matches) {
                pa.push_back(ptsA[kpA[qi]]);
                pb.push_back(ptsB[kpB[ti]]);
            }
            std::vector<char> mask(matches.size(), 1);
            if(matches.size() >= 8) {
                std::vector<uchar> fmask;
                cv::findFundamentalMat(pa, pb, cv::USAC_MAGSAC, 1.5, 0.999, 100000, fmask);
                if(fmask.size() == matches.size()) {
                    for(size_t i = 0; i < matches.size(); ++i) mask[i] = fmask[i];
                }
            }
            for(size_t i = 0; i < matches.size(); ++i) {
                if(!mask[i]) continue;
                metricA.push_back(aPts[matches[i].first]);
                metricB.push_back(bPts[matches[i].second]);
            }
        };
        associate(d1, aKp1, p1, d3, bKp3, p3);  // CAM_B (left) link
        associate(d2, aKp2, p2, d4, bKp4, p4);  // CAM_C (right) link
    }

    result.correspondences = static_cast<int>(metricA.size());
    if(result.correspondences < params.minCorrespondences) {
        result.note = "insufficient four-view metric correspondences; the devices do not share enough co-triangulable, textured scene";
        return result;
    }

    std::vector<double> ranges(metricA.size());
    for(size_t i = 0; i < metricA.size(); ++i) ranges[i] = cv::norm(cv::Vec3d(metricA[i].x, metricA[i].y, metricA[i].z));
    std::nth_element(ranges.begin(), ranges.begin() + ranges.size() / 2, ranges.end());
    const double medianRange = ranges[ranges.size() / 2];
    const double threshold = std::max(0.20, 0.06 * medianRange);

    const RigidFit fit = kabschRansac(metricA, metricB, threshold, 2000, params.ransacSeed);
    if(!fit.ok) {
        result.note = "robust 3D-3D alignment did not converge";
        return result;
    }
    result.inliers = static_cast<int>(std::count(fit.inliers.begin(), fit.inliers.end(), 1));
    result.rmseMeters = fit.rmse;
    result.rotationAFromB = fit.R;
    result.translationAFromB = fit.t;
    result.distanceMeters = cv::norm(fit.t);

    if(result.inliers < params.minInliers || result.rmseMeters > params.maxRmse) {
        result.note = "inter-device scale under-constrained (too few consistent inliers or high residual); reporting unobservable rather than a guess";
        return result;
    }

    result.observable = true;
    result.note = "inter-device scale recovered from four-view metric correspondences";
    return result;
}

}  // namespace impl
}  // namespace dai

#endif  // DEPTHAI_HAVE_OPENCV_SUPPORT
