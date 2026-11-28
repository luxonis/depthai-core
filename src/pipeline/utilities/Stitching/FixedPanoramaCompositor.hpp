#pragma once

#include <opencv2/core.hpp>
#include <opencv2/stitching/detail/blenders.hpp>
#include <opencv2/stitching/detail/exposure_compensate.hpp>
#include <opencv2/stitching/detail/motion_estimators.hpp>
#include <vector>

#include "depthai/pipeline/node/Stitching.hpp"

namespace dai {
namespace utilities {

using node::Stitching;

class FixedPanoramaCompositor {
   public:
    enum class Composition { BLENDED, DIRECT };

    struct Config {
        CameraModel cameraModel = CameraModel::Equirectangular;
        Stitching::SeamFinder seamFinder = Stitching::SeamFinder::GRAPHCUT_COLOR;
        double compositingResolution = -1.0;
        double seamEstimationResolution = 0.1;
        Composition composition = Composition::BLENDED;
    };

    void setConfig(const Config& config);
    void reset();
    bool isPrepared() const;

    void prepare(const std::vector<cv::Mat>& images, const std::vector<cv::detail::CameraParams>& cameras, double registrationScale);

    cv::Mat compose(const std::vector<cv::Mat>& images);

    cv::Size getCanvasSize() const;

    cv::Rect getCanvas() const;  // In warper coordinates; its top-left is panorama pixel (0, 0)

    double getWarperScale() const;  // Radius of the projection surface in pixels

    cv::Mat getSourceMask(size_t sourceIndex) const;

   private:
    struct Source {
        cv::Size inputSize;
        cv::Size composeInputSize;
        cv::Rect roi;
        cv::Mat map1;
        cv::Mat map2;
        cv::Mat mask;
        cv::Mat seamMask;
    };

    std::vector<cv::Mat> warp(const std::vector<cv::Mat>& images) const;
    void prepareCompositing(const std::vector<cv::Mat>& warped);

    Config config;
    bool prepared = false;
    double composeScale = 1.0;
    double warperScale = 0.0;
    cv::Rect canvas;
    std::vector<Source> sources;
    cv::Ptr<cv::detail::ExposureCompensator> compensator;
    cv::Ptr<cv::detail::Blender> blender;
};

}  // namespace utilities
}  // namespace dai
