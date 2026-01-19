#pragma once

#include <cstdint>
#include <string>
#include <vector>

#include "depthai/common/Point2f.hpp"
#include "depthai/common/Point3f.hpp"

// project
#include "depthai/common/KeypointsListT.hpp"
#include "depthai/utility/Serialization.hpp"

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
     * @param spatialCoordinates vector of Point3f objects to set as spatial coordinates.
     * @note The size of spatialCoordinates must match the number of keypoints.
     */
    void setSpatialCoordinates(const std::vector<Point3f>& spatialCoordinates) {
        if(spatialCoordinates.size() != keypoints.size()) {
            throw std::invalid_argument("Size of spatialCoordinates must match the number of keypoints.");
        }
        for(size_t i = 0; i < keypoints.size(); ++i) {
            keypoints[i].spatialCoordinates = spatialCoordinates[i];
        }
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

    DEPTHAI_SERIALIZE(SpatialKeypointsList, keypoints, edges);
};

}  // namespace dai
