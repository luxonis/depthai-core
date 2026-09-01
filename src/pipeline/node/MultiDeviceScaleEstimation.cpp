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

namespace dai {
namespace impl {

namespace {

cv::Mat toGray(const cv::Mat& image) {
    if(image.empty() || image.channels() == 1) return image;
    cv::Mat gray;
    cv::cvtColor(image, gray, cv::COLOR_BGR2GRAY);
    return gray;
}

cv::Mat intrinsicsToMat(const cv::Matx33d& k) {
    return cv::Mat(k);
}

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
            transformed[i] =
                cv::Point2f(static_cast<float>(affine(0, 0) * static_cast<double>(corners[i].x) + affine(0, 1) * static_cast<double>(corners[i].y)),
                            static_cast<float>(affine(1, 0) * static_cast<double>(corners[i].x) + affine(1, 1) * static_cast<double>(corners[i].y)));
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

void asiftDetect(const cv::Mat& gray, bool useAsift, double dedupPx, std::vector<cv::Point2d>& points, cv::Mat& descriptors) {
    cv::Ptr<cv::SIFT> detector = cv::SIFT::create(0, 3, 0.03, 12);

    std::vector<std::pair<double, double>> params = {{1.0, 0.0}};
    if(useAsift) {
        for(int i = 1; i < 4; ++i) {
            const double tilt = std::pow(2.0, 0.5 * i);
            for(double phi = 0.0; phi < 180.0; phi += 72.0 / tilt) params.emplace_back(tilt, phi);
        }
    }

    std::vector<cv::Point2d> allPoints;
    std::vector<float> allResponses;
    cv::Mat allDescriptors;
    for(const auto& [tilt, phi] : params) {
        cv::Matx23d inverseAffine;
        const cv::Mat warped = affineSkew(tilt, phi, gray, inverseAffine);
        std::vector<cv::KeyPoint> keypoints;
        cv::Mat currentDescriptors;
        detector->detectAndCompute(warped, cv::noArray(), keypoints, currentDescriptors);
        if(currentDescriptors.empty()) continue;
        for(size_t i = 0; i < keypoints.size(); ++i) {
            const double x = keypoints[i].pt.x;
            const double y = keypoints[i].pt.y;
            allPoints.emplace_back(inverseAffine(0, 0) * x + inverseAffine(0, 1) * y + inverseAffine(0, 2),
                                   inverseAffine(1, 0) * x + inverseAffine(1, 1) * y + inverseAffine(1, 2));
            allResponses.push_back(keypoints[i].response);
            allDescriptors.push_back(currentDescriptors.row(static_cast<int>(i)));
        }
    }

    points.clear();
    descriptors.release();
    if(allPoints.empty()) return;

    std::vector<int> order(allPoints.size());
    std::iota(order.begin(), order.end(), 0);
    std::sort(order.begin(), order.end(), [&allResponses](int a, int b) { return allResponses[a] > allResponses[b]; });

    std::map<std::pair<long long, long long>, bool> seen;
    std::vector<int> keep;
    for(const int index : order) {
        const std::pair<long long, long long> key{llround(allPoints[index].x / dedupPx), llround(allPoints[index].y / dedupPx)};
        if(seen.count(key)) continue;
        seen[key] = true;
        keep.push_back(index);
    }
    std::sort(keep.begin(), keep.end());

    descriptors.create(static_cast<int>(keep.size()), allDescriptors.cols, allDescriptors.type());
    for(size_t i = 0; i < keep.size(); ++i) {
        points.push_back(allPoints[keep[i]]);
        allDescriptors.row(keep[i]).copyTo(descriptors.row(static_cast<int>(i)));
    }
}

cv::Mat selectRows(const cv::Mat& descriptors, const std::vector<int>& indices) {
    cv::Mat result(static_cast<int>(indices.size()), descriptors.cols, descriptors.type());
    for(size_t i = 0; i < indices.size(); ++i) descriptors.row(indices[i]).copyTo(result.row(static_cast<int>(i)));
    return result;
}

std::vector<std::pair<int, int>> crossMatch(const cv::Mat& first, const cv::Mat& second, double ratio) {
    std::vector<std::pair<int, int>> result;
    if(first.rows < 2 || second.rows < 2) return result;

    cv::FlannBasedMatcher matcher(cv::makePtr<cv::flann::KDTreeIndexParams>(4), cv::makePtr<cv::flann::SearchParams>(32));
    const auto goodMatches = [ratio](const std::vector<std::vector<cv::DMatch>>& nearest) {
        std::map<int, std::pair<int, float>> good;
        for(const auto& pair : nearest) {
            if(pair.size() < 2 || static_cast<double>(pair[0].distance) >= ratio * static_cast<double>(pair[1].distance)) continue;
            const auto it = good.find(pair[0].queryIdx);
            if(it == good.end() || pair[0].distance < it->second.second) good[pair[0].queryIdx] = {pair[0].trainIdx, pair[0].distance};
        }
        return good;
    };

    std::vector<std::vector<cv::DMatch>> forwardNearest, reverseNearest;
    matcher.knnMatch(first, second, forwardNearest, 2);
    matcher.knnMatch(second, first, reverseNearest, 2);
    const auto forward = goodMatches(forwardNearest);
    const auto reverse = goodMatches(reverseNearest);
    for(const auto& [query, trainDistance] : forward) {
        const auto reverseIt = reverse.find(trainDistance.first);
        if(reverseIt != reverse.end() && reverseIt->second.first == query) result.emplace_back(query, trainDistance.first);
    }
    return result;
}

std::vector<cv::Point2d> normalizePoints(const std::vector<cv::Point2d>& pixels, const cv::Matx33d& k) {
    if(pixels.empty()) return {};
    std::vector<cv::Point2d> normalized;
    cv::undistortPoints(pixels, normalized, intrinsicsToMat(k), cv::noArray());
    return normalized;
}

std::vector<cv::Point2d> projectPoints(const std::vector<cv::Point3d>& points, const cv::Matx33d& k) {
    std::vector<cv::Point2d> projected(points.size(), cv::Point2d(std::nan(""), std::nan("")));
    std::vector<cv::Point3d> valid;
    std::vector<int> validIndices;
    for(size_t i = 0; i < points.size(); ++i) {
        const auto& point = points[i];
        if(std::isfinite(point.x) && std::isfinite(point.y) && std::isfinite(point.z) && point.z > 1e-8) {
            valid.push_back(point);
            validIndices.push_back(static_cast<int>(i));
        }
    }
    if(valid.empty()) return projected;
    std::vector<cv::Point2d> validProjected;
    cv::projectPoints(valid, cv::Vec3d(0, 0, 0), cv::Vec3d(0, 0, 0), intrinsicsToMat(k), cv::noArray(), validProjected);
    for(size_t i = 0; i < validIndices.size(); ++i) projected[validIndices[i]] = validProjected[i];
    return projected;
}

cv::Matx44d invertSe3(const cv::Matx44d& transform) {
    cv::Matx33d rotation;
    cv::Vec3d translation;
    for(int i = 0; i < 3; ++i) {
        for(int j = 0; j < 3; ++j) rotation(i, j) = transform(i, j);
        translation(i) = transform(i, 3);
    }
    const auto inverseRotation = rotation.t();
    const auto inverseTranslation = -(inverseRotation * translation);
    cv::Matx44d result = cv::Matx44d::eye();
    for(int i = 0; i < 3; ++i) {
        for(int j = 0; j < 3; ++j) result(i, j) = inverseRotation(i, j);
        result(i, 3) = inverseTranslation(i);
    }
    return result;
}

}  // namespace

void triangulateMetric(const std::vector<cv::Point2d>& leftPixels,
                       const std::vector<cv::Point2d>& rightPixels,
                       const cv::Matx33d& intrinsicsLeft,
                       const cv::Matx33d& intrinsicsRight,
                       const cv::Matx44d& leftFromRight,
                       double gatePx,
                       std::vector<cv::Point3d>& pointsLeft,
                       std::vector<char>& keep) {
    pointsLeft.assign(leftPixels.size(), cv::Point3d(std::nan(""), std::nan(""), std::nan("")));
    keep.assign(leftPixels.size(), 0);
    if(leftPixels.empty() || leftPixels.size() != rightPixels.size()) return;

    const auto leftNormalized = normalizePoints(leftPixels, intrinsicsLeft);
    const auto rightNormalized = normalizePoints(rightPixels, intrinsicsRight);
    const auto rightFromLeft = invertSe3(leftFromRight);
    cv::Matx34d leftProjection(1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0);
    cv::Matx34d rightProjection;
    for(int i = 0; i < 3; ++i)
        for(int j = 0; j < 4; ++j) rightProjection(i, j) = rightFromLeft(i, j);

    cv::Mat leftPoints(2, static_cast<int>(leftNormalized.size()), CV_64F);
    cv::Mat rightPoints(2, static_cast<int>(rightNormalized.size()), CV_64F);
    for(size_t i = 0; i < leftNormalized.size(); ++i) {
        leftPoints.at<double>(0, static_cast<int>(i)) = leftNormalized[i].x;
        leftPoints.at<double>(1, static_cast<int>(i)) = leftNormalized[i].y;
        rightPoints.at<double>(0, static_cast<int>(i)) = rightNormalized[i].x;
        rightPoints.at<double>(1, static_cast<int>(i)) = rightNormalized[i].y;
    }

    cv::Mat points4d;
    cv::triangulatePoints(cv::Mat(leftProjection), cv::Mat(rightProjection), leftPoints, rightPoints, points4d);
    const auto rightRotation = rightFromLeft.get_minor<3, 3>(0, 0);
    const cv::Vec3d rightTranslation(rightFromLeft(0, 3), rightFromLeft(1, 3), rightFromLeft(2, 3));
    std::vector<cv::Point3d> pointsRight(leftPixels.size(), cv::Point3d(std::nan(""), std::nan(""), std::nan("")));
    for(int i = 0; i < points4d.cols; ++i) {
        const double w = points4d.at<double>(3, i);
        if(std::abs(w) <= 1e-8) continue;
        const cv::Point3d pointLeft(points4d.at<double>(0, i) / w, points4d.at<double>(1, i) / w, points4d.at<double>(2, i) / w);
        pointsLeft[i] = pointLeft;
        const auto pointRight = rightRotation * cv::Vec3d(pointLeft.x, pointLeft.y, pointLeft.z) + rightTranslation;
        pointsRight[i] = cv::Point3d(pointRight(0), pointRight(1), pointRight(2));
    }

    const auto reprojectionLeft = projectPoints(pointsLeft, intrinsicsLeft);
    const auto reprojectionRight = projectPoints(pointsRight, intrinsicsRight);
    for(size_t i = 0; i < leftPixels.size(); ++i) {
        const auto& pointLeft = pointsLeft[i];
        const auto& pointRight = pointsRight[i];
        const bool positiveDepth = std::isfinite(pointLeft.z) && pointLeft.z > 0.0 && std::isfinite(pointRight.z) && pointRight.z > 0.0;
        if(!positiveDepth) continue;
        const double leftError = cv::norm(reprojectionLeft[i] - leftPixels[i]);
        const double rightError = cv::norm(reprojectionRight[i] - rightPixels[i]);
        const double error = std::max(leftError, rightError);
        if(std::isfinite(error) && error <= gatePx) keep[i] = 1;
    }
}

bool kabsch(const std::vector<cv::Point3d>& pointsA, const std::vector<cv::Point3d>& pointsB, cv::Matx33d& rotation, cv::Vec3d& translation) {
    if(pointsA.size() < 3 || pointsA.size() != pointsB.size()) return false;
    cv::Vec3d centroidA(0, 0, 0), centroidB(0, 0, 0);
    for(size_t i = 0; i < pointsA.size(); ++i) {
        centroidA += cv::Vec3d(pointsA[i].x, pointsA[i].y, pointsA[i].z);
        centroidB += cv::Vec3d(pointsB[i].x, pointsB[i].y, pointsB[i].z);
    }
    centroidA *= 1.0 / static_cast<double>(pointsA.size());
    centroidB *= 1.0 / static_cast<double>(pointsB.size());

    cv::Matx33d covariance = cv::Matx33d::zeros();
    for(size_t i = 0; i < pointsA.size(); ++i) {
        const auto a = cv::Vec3d(pointsA[i].x, pointsA[i].y, pointsA[i].z) - centroidA;
        const auto b = cv::Vec3d(pointsB[i].x, pointsB[i].y, pointsB[i].z) - centroidB;
        covariance += b * a.t();
    }

    cv::Matx33d u, vt;
    cv::Matx31d singular;
    cv::SVD::compute(covariance, singular, u, vt);
    const auto v = vt.t();
    const double determinantSign = cv::determinant(v * u.t()) < 0.0 ? -1.0 : 1.0;
    const cv::Matx33d correction(1, 0, 0, 0, 1, 0, 0, 0, determinantSign);
    rotation = v * correction * u.t();
    translation = centroidA - rotation * centroidB;
    return true;
}

RigidFit kabschRansac(const std::vector<cv::Point3d>& pointsA, const std::vector<cv::Point3d>& pointsB, double threshold, int iterations, unsigned seed) {
    RigidFit fit;
    const int count = static_cast<int>(pointsA.size());
    if(count < 3 || pointsA.size() != pointsB.size()) return fit;

    std::mt19937 generator(seed);
    std::uniform_int_distribution<int> pick(0, count - 1);
    const auto residuals = [&](const cv::Matx33d& rotation, const cv::Vec3d& translation) {
        std::vector<double> result(count);
        for(int i = 0; i < count; ++i) {
            const cv::Vec3d a(pointsA[i].x, pointsA[i].y, pointsA[i].z);
            const cv::Vec3d b(pointsB[i].x, pointsB[i].y, pointsB[i].z);
            result[i] = cv::norm(a - (rotation * b + translation));
        }
        return result;
    };

    std::vector<char> bestInliers;
    int bestCount = 0;
    for(int iteration = 0; iteration < iterations; ++iteration) {
        const int i0 = pick(generator), i1 = pick(generator), i2 = pick(generator);
        if(i0 == i1 || i1 == i2 || i0 == i2) continue;
        const std::vector<cv::Point3d> sampleA{pointsA[i0], pointsA[i1], pointsA[i2]};
        const std::vector<cv::Point3d> sampleB{pointsB[i0], pointsB[i1], pointsB[i2]};
        cv::Matx33d rotation;
        cv::Vec3d translation;
        if(!kabsch(sampleA, sampleB, rotation, translation)) continue;
        const auto errors = residuals(rotation, translation);
        std::vector<char> inliers(count, 0);
        int inlierCount = 0;
        for(int i = 0; i < count; ++i) {
            inliers[i] = errors[i] < threshold ? 1 : 0;
            inlierCount += inliers[i];
        }
        if(inlierCount > bestCount) {
            bestCount = inlierCount;
            bestInliers = std::move(inliers);
        }
    }
    if(bestInliers.empty() || bestCount < 3) return fit;

    std::vector<cv::Point3d> inlierA, inlierB;
    for(int i = 0; i < count; ++i) {
        if(bestInliers[i]) {
            inlierA.push_back(pointsA[i]);
            inlierB.push_back(pointsB[i]);
        }
    }
    cv::Matx33d rotation;
    cv::Vec3d translation;
    if(!kabsch(inlierA, inlierB, rotation, translation)) return fit;

    for(int iteration = 0; iteration < 10 && inlierA.size() >= 3; ++iteration) {
        std::vector<double> errors(inlierA.size());
        for(size_t i = 0; i < inlierA.size(); ++i) {
            const cv::Vec3d a(inlierA[i].x, inlierA[i].y, inlierA[i].z);
            const cv::Vec3d b(inlierB[i].x, inlierB[i].y, inlierB[i].z);
            errors[i] = cv::norm(a - (rotation * b + translation));
        }
        auto sortedErrors = errors;
        std::nth_element(sortedErrors.begin(), sortedErrors.begin() + sortedErrors.size() / 2, sortedErrors.end());
        const double medianError = sortedErrors[sortedErrors.size() / 2];
        const double huber = std::max(1.345 * medianError, 1e-3);

        std::vector<double> weights(inlierA.size());
        double weightSum = 0.0;
        for(size_t i = 0; i < inlierA.size(); ++i) {
            weights[i] = errors[i] > huber ? huber / errors[i] : 1.0;
            weightSum += weights[i];
        }
        cv::Vec3d centroidA(0, 0, 0), centroidB(0, 0, 0);
        for(size_t i = 0; i < inlierA.size(); ++i) {
            const double weight = weights[i] / weightSum;
            centroidA += weight * cv::Vec3d(inlierA[i].x, inlierA[i].y, inlierA[i].z);
            centroidB += weight * cv::Vec3d(inlierB[i].x, inlierB[i].y, inlierB[i].z);
        }
        cv::Matx33d covariance = cv::Matx33d::zeros();
        for(size_t i = 0; i < inlierA.size(); ++i) {
            const auto a = cv::Vec3d(inlierA[i].x, inlierA[i].y, inlierA[i].z) - centroidA;
            const auto b = cv::Vec3d(inlierB[i].x, inlierB[i].y, inlierB[i].z) - centroidB;
            covariance += weights[i] * (b * a.t());
        }
        cv::Matx33d u, vt;
        cv::Matx31d singular;
        cv::SVD::compute(covariance, singular, u, vt);
        const auto v = vt.t();
        const double determinantSign = cv::determinant(v * u.t()) < 0.0 ? -1.0 : 1.0;
        const cv::Matx33d correction(1, 0, 0, 0, 1, 0, 0, 0, determinantSign);
        rotation = v * correction * u.t();
        translation = centroidA - rotation * centroidB;
    }

    double squaredError = 0.0;
    for(size_t i = 0; i < inlierA.size(); ++i) {
        const cv::Vec3d a(inlierA[i].x, inlierA[i].y, inlierA[i].z);
        const cv::Vec3d b(inlierB[i].x, inlierB[i].y, inlierB[i].z);
        const double error = cv::norm(a - (rotation * b + translation));
        squaredError += error * error;
    }
    fit.R = rotation;
    fit.t = translation;
    fit.inliers = std::move(bestInliers);
    fit.rmse = inlierA.empty() ? std::numeric_limits<double>::infinity() : std::sqrt(squaredError / inlierA.size());
    fit.ok = true;
    return fit;
}

namespace {

std::vector<cv::Mat> undistortAll(const std::vector<cv::Mat>& images, const cv::Matx33d& k, const std::vector<double>& distortion) {
    std::vector<cv::Mat> result;
    result.reserve(images.size());
    const cv::Mat distortionMat = distortion.empty() ? cv::Mat() : cv::Mat(distortion).clone();
    for(const auto& image : images) {
        cv::Mat undistorted;
        cv::undistort(toGray(image), undistorted, intrinsicsToMat(k), distortionMat);
        result.push_back(std::move(undistorted));
    }
    return result;
}

}  // namespace

InterDeviceScaleResult estimateInterDeviceScale(const StereoDeviceViews& deviceA, const StereoDeviceViews& deviceB, const ScaleParams& params) {
    InterDeviceScaleResult result;
    const size_t frames = std::min({deviceA.left.size(), deviceA.right.size(), deviceB.left.size(), deviceB.right.size()});
    if(frames == 0) {
        result.note = "no synchronized frames to estimate the inter-device scale from";
        return result;
    }

    const auto aLeft = undistortAll(deviceA.left, deviceA.intrinsicsLeft, deviceA.distortionLeft);
    const auto aRight = undistortAll(deviceA.right, deviceA.intrinsicsRight, deviceA.distortionRight);
    const auto bLeft = undistortAll(deviceB.left, deviceB.intrinsicsLeft, deviceB.distortionLeft);
    const auto bRight = undistortAll(deviceB.right, deviceB.intrinsicsRight, deviceB.distortionRight);

    std::vector<cv::Point3d> metricA;
    std::vector<cv::Point3d> metricB;
    for(size_t frameIndex = 0; frameIndex < frames; ++frameIndex) {
        std::vector<cv::Point2d> points1, points2, points3, points4;
        cv::Mat descriptors1, descriptors2, descriptors3, descriptors4;
        asiftDetect(aLeft[frameIndex], params.useAsift, params.dedupPx, points1, descriptors1);
        asiftDetect(aRight[frameIndex], params.useAsift, params.dedupPx, points2, descriptors2);
        asiftDetect(bLeft[frameIndex], params.useAsift, params.dedupPx, points3, descriptors3);
        asiftDetect(bRight[frameIndex], params.useAsift, params.dedupPx, points4, descriptors4);

        const auto matchesA = crossMatch(descriptors1, descriptors2, params.ratio);
        const auto matchesB = crossMatch(descriptors3, descriptors4, params.ratio);
        if(matchesA.empty() || matchesB.empty()) continue;

        std::vector<cv::Point2d> a1, a2, b3, b4;
        for(const auto& [query, train] : matchesA) {
            a1.push_back(points1[query]);
            a2.push_back(points2[train]);
        }
        for(const auto& [query, train] : matchesB) {
            b3.push_back(points3[query]);
            b4.push_back(points4[train]);
        }

        std::vector<cv::Point3d> xA, xB;
        std::vector<char> keepA, keepB;
        triangulateMetric(a1, a2, deviceA.intrinsicsLeft, deviceA.intrinsicsRight, deviceA.leftFromRight, params.stereoGatePx, xA, keepA);
        triangulateMetric(b3, b4, deviceB.intrinsicsLeft, deviceB.intrinsicsRight, deviceB.leftFromRight, params.stereoGatePx, xB, keepB);

        std::vector<int> aKp1, aKp2;
        std::vector<cv::Point3d> aPoints;
        for(size_t i = 0; i < matchesA.size(); ++i) {
            if(!keepA[i] || xA[i].z < params.depthMin || xA[i].z > params.depthMax) continue;
            aKp1.push_back(matchesA[i].first);
            aKp2.push_back(matchesA[i].second);
            aPoints.push_back(xA[i]);
        }
        std::vector<int> bKp3, bKp4;
        std::vector<cv::Point3d> bPoints;
        for(size_t i = 0; i < matchesB.size(); ++i) {
            if(!keepB[i] || xB[i].z < params.depthMin || xB[i].z > params.depthMax) continue;
            bKp3.push_back(matchesB[i].first);
            bKp4.push_back(matchesB[i].second);
            bPoints.push_back(xB[i]);
        }
        if(aPoints.size() < 4 || bPoints.size() < 4) continue;

        const auto associate = [&](const cv::Mat& allDescriptorsA,
                                   const std::vector<int>& keypointsA,
                                   const std::vector<cv::Point2d>& pixelsA,
                                   const cv::Mat& allDescriptorsB,
                                   const std::vector<int>& keypointsB,
                                   const std::vector<cv::Point2d>& pixelsB,
                                   const std::vector<cv::Point3d>& metricPointsA,
                                   const std::vector<cv::Point3d>& metricPointsB) {
            const auto descriptorsA = selectRows(allDescriptorsA, keypointsA);
            const auto descriptorsB = selectRows(allDescriptorsB, keypointsB);
            const auto matches = crossMatch(descriptorsA, descriptorsB, params.ratio);
            if(matches.empty()) return;
            std::vector<cv::Point2d> projectedA, projectedB;
            for(const auto& [query, train] : matches) {
                projectedA.push_back(pixelsA[keypointsA[query]]);
                projectedB.push_back(pixelsB[keypointsB[train]]);
            }
            std::vector<uchar> mask(matches.size(), 1);
            if(matches.size() >= 8) {
                std::vector<uchar> fundamentalMask;
                cv::findFundamentalMat(projectedA, projectedB, cv::USAC_MAGSAC, 1.5, 0.999, 100000, fundamentalMask);
                if(fundamentalMask.size() == matches.size()) mask = std::move(fundamentalMask);
            }
            for(size_t i = 0; i < matches.size(); ++i) {
                if(mask[i]) {
                    metricA.push_back(metricPointsA[matches[i].first]);
                    metricB.push_back(metricPointsB[matches[i].second]);
                }
            }
        };
        associate(descriptors1, aKp1, points1, descriptors3, bKp3, points3, aPoints, bPoints);
        associate(descriptors2, aKp2, points2, descriptors4, bKp4, points4, aPoints, bPoints);
    }

    result.correspondences = static_cast<int>(metricA.size());
    if(result.correspondences < params.minCorrespondences) {
        result.note = "insufficient four-view metric correspondences; the devices do not share enough co-triangulable, textured scene";
        return result;
    }

    std::vector<double> ranges(metricA.size());
    for(size_t i = 0; i < metricA.size(); ++i) ranges[i] = cv::norm(cv::Vec3d(metricA[i].x, metricA[i].y, metricA[i].z));
    std::nth_element(ranges.begin(), ranges.begin() + ranges.size() / 2, ranges.end());
    const double threshold = std::max(0.20, 0.06 * ranges[ranges.size() / 2]);
    const auto fit = kabschRansac(metricA, metricB, threshold, 2000, params.ransacSeed);
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
