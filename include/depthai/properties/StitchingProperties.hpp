#pragma once

#include <array>
#include <cstdint>
#include <limits>
#include <optional>

#include "depthai/common/DepthUnit.hpp"
#include "depthai/common/Point3f.hpp"
#include "depthai/common/optional.hpp"
#include "depthai/properties/Properties.hpp"

namespace dai {

/** Serializable properties for the Stitching node. */
struct StitchingProperties : PropertiesSerializable<Properties, StitchingProperties> {
    /** Stitching mode. */
    enum class Mode : std::uint8_t {
        /// Photo panorama registered from overlapping image content.
        PANORAMA,
        /// Projection of calibrated views onto a plane.
        PLANAR_PROJECTION,
    };

    /** Camera projection model the panorama images are warped onto. */
    enum class CameraModel : std::uint8_t {
        /// Spherical surface, the OpenCV default.
        SPHERICAL,
        /// Plane, appropriate for a pinhole camera and small fields of view.
        PINHOLE,
        /// Cylindrical surface.
        CYLINDRICAL,
    };

    /** Seam estimation method. */
    enum class SeamFinder : std::uint8_t { NONE, VORONOI, DP_COLOR, DP_COLOR_GRAD, GRAPHCUT_COLOR, GRAPHCUT_COLOR_GRAD };

    /** A plane used by planar projection stitching. */
    struct Plane {
        /// A point lying on the plane.
        Point3f point;
        /// Normal of the plane; it does not have to be of unit length.
        Point3f normal;
        /// Length unit of `point`.
        LengthUnit unit = LengthUnit::CENTIMETER;
    };

    /** The pinhole camera used to render a planar projection. */
    struct VirtualCamera {
        /// Pose of the camera with respect to the reference frame.
        std::array<std::array<float, 4>, 4> pose = {{{1, 0, 0, 0}, {0, 1, 0, 0}, {0, 0, 1, 0}, {0, 0, 0, 1}}};
        /// Length unit of the translation part of `pose`.
        LengthUnit unit = LengthUnit::CENTIMETER;
        /// Intrinsic matrix of the camera.
        std::array<std::array<float, 3>, 3> intrinsics = {{{1, 0, 0}, {0, 1, 0}, {0, 0, 1}}};
        /// Width of the rendered image in pixels.
        std::uint32_t width = 0;
        /// Height of the rendered image in pixels.
        std::uint32_t height = 0;

        /**
         * Build a camera placed at `position` and looking at `target`.
         *
         * @param position Camera center, in the reference frame
         * @param target Point the optical axis passes through, in the reference frame
         * @param up Direction that ends up pointing up in the rendered image
         * @param hFovDegrees Horizontal field of view of the camera
         * @param width Width of the rendered image in pixels
         * @param height Height of the rendered image in pixels
         * @param unit Length unit of `position` and `target`
         */
        static VirtualCamera lookAt(const Point3f& position,
                                    const Point3f& target,
                                    const Point3f& up,
                                    float hFovDegrees,
                                    std::uint32_t width,
                                    std::uint32_t height,
                                    LengthUnit unit = LengthUnit::CENTIMETER);
    };

    Mode mode = Mode::PANORAMA;
    CameraModel cameraModel = CameraModel::SPHERICAL;
    bool continuous = false;
    std::uint32_t estimationFrames = 10;
    std::uint32_t maxPanoramaWidth = std::numeric_limits<std::uint32_t>::max();
    std::uint32_t maxPanoramaHeight = std::numeric_limits<std::uint32_t>::max();
    double panoConfidenceThreshold = 1.0;
    SeamFinder seamFinder = SeamFinder::GRAPHCUT_COLOR;

    std::optional<Plane> plane;
    std::optional<VirtualCamera> view;
    std::uint32_t maxViewWidth = 1920;
    std::uint32_t maxViewHeight = 1920;
    /// Maximum planar projection range, stored in centimeters.
    float maxRange = 1000.0f;
    float minIncidenceAngle = 5.0f;

    ~StitchingProperties() override;
};

DEPTHAI_SERIALIZE_EXT(StitchingProperties::Plane, point, normal, unit);
DEPTHAI_SERIALIZE_EXT(StitchingProperties::VirtualCamera, pose, unit, intrinsics, width, height);
DEPTHAI_SERIALIZE_EXT(StitchingProperties,
                      mode,
                      cameraModel,
                      continuous,
                      estimationFrames,
                      maxPanoramaWidth,
                      maxPanoramaHeight,
                      panoConfidenceThreshold,
                      seamFinder,
                      plane,
                      view,
                      maxViewWidth,
                      maxViewHeight,
                      maxRange,
                      minIncidenceAngle);

}  // namespace dai
