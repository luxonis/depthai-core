#pragma once

#include <cstdint>
#include <opencv2/core.hpp>
#include <opencv2/stitching/detail/exposure_compensate.hpp>
#include <optional>
#include <vector>

#include "depthai/common/ImgTransformations.hpp"
#include "depthai/pipeline/node/Stitching.hpp"

namespace dai {
namespace utilities {

using node::Stitching;

class PlanarStitcher {  // Inputs must share one origin socket; geometry is built from the first group, reset() rebuilds
   public:
    struct Config {
        std::optional<Stitching::Plane> plane;         // Required
        std::optional<Stitching::VirtualCamera> view;  // Computed from the views when not set
        uint32_t maxViewWidth = 1920;                  // Of a computed view only, in pixels
        uint32_t maxViewHeight = 1920;
        float maxRange = 1000.0f;        // Centimeters from a camera center; the plane is not painted beyond
        float minIncidenceAngle = 5.0f;  // Degrees between a camera ray and the plane
        Stitching::SeamFinder seamFinder = Stitching::SeamFinder::GRAPHCUT_COLOR;
    };

    void setConfig(const Config& config);  // Drops everything built from the previous config
    const Config& getConfig() const;

    void reset();

    bool isPrepared() const;

    void prepare(const std::vector<ImgTransformation>& transformations);  // Throws runtime_error if inputs are inconsistent or none sees the plane

    void validateTransformations(const std::vector<ImgTransformation>& transformations) const;

    cv::Mat compose(const std::vector<cv::Mat>& images);  // Images in prepare() order, also ones that miss the plane; returns BGR of the view size

    const Stitching::VirtualCamera& getResolvedView() const;  // Valid after prepare()

    CameraBoardSocket getOrigin() const;  // Frame of the plane, view and extrinsics; valid after prepare()

   private:
    struct Source {
        size_t index = 0;
        cv::Mat map1;  // Fixed-point maps of roi from cv::convertMaps
        cv::Mat map2;
        cv::Mat mask;
        cv::Mat seamMask;
        cv::Rect roi;
        cv::Size imageSize;
    };

    struct SourcePose {  // In the reference frame, in centimeters
        cv::Matx33d rotation;
        cv::Vec3d center;
    };

    Stitching::VirtualCamera computeView(const std::vector<ImgTransformation>& transformations, const std::vector<SourcePose>& poses) const;

    void prepareCompositing(const std::vector<cv::Mat>& warped);

    Config config;

    bool prepared = false;
    bool compositingPrepared = false;
    size_t numInputs = 0;  // Includes inputs that do not see the plane
    std::vector<ImgTransformation> preparedTransformations;
    CameraBoardSocket origin = CameraBoardSocket::AUTO;
    Stitching::VirtualCamera view;
    cv::Vec3d planePoint;  // Reference frame, centimeters; the normal is a unit vector
    cv::Vec3d planeNormal;
    std::vector<Source> sources;
    cv::Ptr<cv::detail::ExposureCompensator> compensator;
};

}  // namespace utilities
}  // namespace dai
