#pragma once

#include <cstdint>
#include <vector>

#include "depthai/common/Point2f.hpp"
#include "depthai/common/Point3f.hpp"

// project
#include "depthai/common/KeypointsListT.hpp"
#include "depthai/utility/Serialization.hpp"

namespace dai {

struct Keypoint {
    Point3f imageCoordinates;
    float confidence = -1.f;  // -1.f indicates confidence is not set
    uint32_t label = 0;
    std::string labelName = "";

    Keypoint() = default;
    /**
     * Construct a keypoint from 3D image coordinates.
     * @throws std::invalid_argument if confidence is negative.
     */
    explicit Keypoint(Point3f imageCoordinates, float conf = 0.f, uint32_t label = 0, std::string labelName = "")
        : imageCoordinates(imageCoordinates), confidence(conf), label(label), labelName(labelName) {
        if(confidence < 0.f) {
            throw std::invalid_argument("Confidence must be non-negative.");
        }
    }

    /**
     * Construct a keypoint from 2D image coordinates (z = 0).
     */
    explicit Keypoint(Point2f imageCoordinates, float confidence = 0.f, uint32_t label = 0, std::string labelName = "")
        : Keypoint(Point3f{imageCoordinates.x, imageCoordinates.y, 0.f}, confidence, label, labelName) {}

    /**
     * Construct a keypoint from explicit x/y/z coordinates.
     */
    explicit Keypoint(float x, float y, float z, float confidence = 0.f, uint32_t label = 0, std::string labelName = "")
        : Keypoint(Point3f{x, y, z}, confidence, label, labelName) {}

    DEPTHAI_SERIALIZE(dai::Keypoint, imageCoordinates, confidence, label, labelName);
};

struct KeypointsList : KeypointsListT<Keypoint> {
   public:
    using Base = KeypointsListT<Keypoint>;
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
    void setKeypoints(const std::vector<Point3f> kps3) {
        edges.clear();
        keypoints.clear();
        keypoints.reserve(kps3.size());
        for(const auto& kp : kps3) {
            keypoints.emplace_back(Keypoint(kp));
        }
    }

    /**
     * Sets the keypoints list.
     * @param keypoints list of Point2f objects to set.
     * @note This will clear any existing keypoints and edges.
     */
    void setKeypoints(const std::vector<Point2f> kps2) {
        edges.clear();
        keypoints.clear();
        keypoints.reserve(kps2.size());
        for(const auto& kp : kps2) {
            keypoints.emplace_back(Keypoint(kp));
        }
    }

    DEPTHAI_SERIALIZE(KeypointsList, keypoints, edges);
};

}  // namespace dai
