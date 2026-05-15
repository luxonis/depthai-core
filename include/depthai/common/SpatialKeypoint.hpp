#pragma once

#include <cstdint>
#include <string>
#include <vector>

#include "depthai/common/DepthUnit.hpp"
#include "depthai/common/ImgTransformations.hpp"
#include "depthai/common/Point2f.hpp"
#include "depthai/common/Point3f.hpp"

// project
#include "depthai/common/KeypointsListT.hpp"
#include "depthai/utility/Serialization.hpp"
#include "depthai/utility/matrixOps.hpp"

namespace dai {

struct SpatialKeypoint {
    Point3f imageCoordinates{};
    float confidence = -1.f;
    uint32_t label = 0;
    std::string labelName = "";
    Point3f spatialCoordinates{};

    SpatialKeypoint() = default;
    explicit SpatialKeypoint(Point3f imageCoordinates, Point3f spatialCoordinates = Point3f{}, float conf = 0.f, uint32_t label = 0, std::string labelName = "")
        : imageCoordinates(imageCoordinates), confidence(conf), label(label), labelName(labelName), spatialCoordinates(spatialCoordinates) {}

    explicit SpatialKeypoint(Point2f imageCoordinates, Point3f spatialCoordinates = Point3f{}, float conf = 0.f, uint32_t label = 0, std::string labelName = "")
        : SpatialKeypoint(Point3f(imageCoordinates.x, imageCoordinates.y, 0.f), spatialCoordinates, conf, label, labelName) {}
    explicit SpatialKeypoint(float x, float y, float z, float sx, float sy, float sz, float conf = 0.f, uint32_t label = 0, std::string labelName = "")
        : SpatialKeypoint(Point3f(x, y, z), Point3f{sx, sy, sz}, conf, label, labelName) {}

    DEPTHAI_SERIALIZE(dai::SpatialKeypoint, imageCoordinates, confidence, label, labelName, spatialCoordinates);
};

/**
 * List of spatial keypoints.
 *
 * All `SpatialKeypoint::spatialCoordinates` values stored in this list are expressed in `unit` units. Mixed spatial units within a single list are not
 * supported.
 */
struct SpatialKeypointsList : KeypointsListT<SpatialKeypoint> {
   public:
    using Base = KeypointsListT<SpatialKeypoint>;
    using Base::Base;
    using Base::edges;
    using Base::getEdges;
    using Base::getKeypoints;
    using Base::getPoints2f;
    using Base::getPoints3f;
    using Base::keypoints;
    using Base::setEdges;
    using Base::setKeypoints;

    /**
     * Length unit used by all keypoints' `spatialCoordinates` in this list.
     */
    LengthUnit unit = LengthUnit::MILLIMETER;

    explicit SpatialKeypointsList(std::vector<SpatialKeypoint> keypoints, std::vector<Edge> edges, LengthUnit unit = LengthUnit::MILLIMETER)
        : Base(std::move(keypoints), std::move(edges)), unit(unit) {
        validateEdges();
    }

    explicit SpatialKeypointsList(std::vector<SpatialKeypoint> keypoints, LengthUnit unit = LengthUnit::MILLIMETER) : Base(std::move(keypoints)), unit(unit) {}

    /**
     * Sets the keypoints list.
     * @param keypoints list of Point3f objects to set.
     * @note This will clear any existing keypoints and edges.
     */
    void setKeypoints(const std::vector<Point3f>& kps3) {
        edges.clear();
        keypoints.clear();
        keypoints.reserve(kps3.size());
        for(const auto& kp : kps3) {
            keypoints.emplace_back(SpatialKeypoint(kp));
        }
    }

    /**
     * Sets the keypoints from a vector of 3D spatial points.
     *
     * The size of spatialCoordinates must match the number of keypoints.
     *
     * @param spatialCoordinates vector of Point3f objects to set as spatial coordinates.
     * @param spatialUnit The unit of the spatial coordinates. By default, spatial coordinates are in millimeters.
     */
    void setSpatialCoordinates(const std::vector<Point3f>& spatialCoordinates, LengthUnit spatialUnit = LengthUnit::MILLIMETER) {
        if(spatialCoordinates.size() != keypoints.size()) {
            throw std::invalid_argument("Size of spatialCoordinates must match the number of keypoints.");
        }
        for(size_t i = 0; i < keypoints.size(); ++i) {
            keypoints[i].spatialCoordinates = spatialCoordinates[i];
        }
        unit = spatialUnit;
    }

    /**
     * Get spatial coordinates of the keypoints.
     * @return Vector of Point3f spatial coordinates.
     */
    std::vector<Point3f> getSpatialCoordinates() const {
        std::vector<Point3f> spatialCoordinates;
        spatialCoordinates.reserve(keypoints.size());
        for(const auto& kp : keypoints) {
            spatialCoordinates.emplace_back(kp.spatialCoordinates);
        }
        return spatialCoordinates;
    }

    /**
     * Transforms the list of keypoints to the target image transformation under the assumption that the keypoints are defined in the source image
     * transformation.
     *
     * If a keypoint does not have valid spatial coordinates (`spatialCoordinates.z <= 0`), the depth-based 3D projection is skipped and
     * only a 2D remapping is applied to its imageCoordinates.
     *
     * @param source Source image transformation.
     * @param target Target image transformation.
     */
    SpatialKeypointsList transformTo(const ImgTransformation& source, const ImgTransformation& target) const {
        SpatialKeypointsList transformed = *this;

        const auto transMatrix = source.getExtrinsicsTransformationMatrixTo(target, false, transformed.unit);
        float depthScale = getDistanceUnitScale(LengthUnit::MILLIMETER, transformed.unit);
        for(auto& kp : transformed.keypoints) {
            Point2f imgCoordinates2f{kp.imageCoordinates.x, kp.imageCoordinates.y};
            if(kp.spatialCoordinates.z > 0) {
                float depth = kp.spatialCoordinates.z * depthScale;
                // TODO (aljazkonec1): projecting each point seperately is inefficient, possible optimization by loading all the necessary matrixes once and
                // reusing them.
                Point2f remappedImgCoordinates = source.projectPointTo(target, imgCoordinates2f, depth);
                kp.imageCoordinates.x = remappedImgCoordinates.x;
                kp.imageCoordinates.y = remappedImgCoordinates.y;

                kp.spatialCoordinates = matrix::transformPoint3f(transMatrix, kp.spatialCoordinates);
            } else {
                Point2f remappedPoint = source.remapPointTo(target, Point2f{kp.imageCoordinates.x, kp.imageCoordinates.y});
                kp.imageCoordinates.x = remappedPoint.x;
                kp.imageCoordinates.y = remappedPoint.y;
            }
        }
        return transformed;
    }

    DEPTHAI_SERIALIZE(SpatialKeypointsList, keypoints, edges, unit);
};

}  // namespace dai
