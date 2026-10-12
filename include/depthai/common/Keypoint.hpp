#pragma once

#include <cstdint>
#include <vector>

#include "depthai/common/ImgTransformations.hpp"
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
    std::string labelName;

    Keypoint() = default;
    explicit Keypoint(Point3f imageCoordinates, float conf = 0.f, uint32_t label = 0, std::string labelName = "")
        : imageCoordinates(imageCoordinates), confidence(conf), label(label), labelName(std::move(labelName)) {}

    explicit Keypoint(Point2f imageCoordinates, float confidence = 0.f, uint32_t label = 0, std::string labelName = "")
        : Keypoint(Point3f{imageCoordinates.x, imageCoordinates.y, 0.f}, confidence, label, std::move(labelName)) {}

    explicit Keypoint(float x, float y, float z, float confidence = 0.f, uint32_t label = 0, std::string labelName = "")
        : Keypoint(Point3f{x, y, z}, confidence, label, std::move(labelName)) {}

    void transform(const ImgTransformation& source, const ImgTransformation& target) {
        // TODO (aljazkonec1) Possible issue: Keypoint does not store its own normalization flag, so normalization is guessed based on if coordinates are in
        // range [0,1]. A keypoint can be normalized but outside the [0, 1] range. Possible fix: Add hasNormalized and normalized fields to KeypointsList and
        // Keypoint.
        Point2f remappedPoint = source.remapPointTo(target, Point2f{imageCoordinates.x, imageCoordinates.y});
        imageCoordinates.x = remappedPoint.x;
        imageCoordinates.y = remappedPoint.y;
    }

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
    void setKeypoints(const std::vector<Point3f>& kps3) {
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
    void setKeypoints(const std::vector<Point2f>& kps2) {
        edges.clear();
        keypoints.clear();
        keypoints.reserve(kps2.size());
        for(const auto& kp : kps2) {
            keypoints.emplace_back(Keypoint(kp));
        }
    }

    /**
     * Returns a new KeypointsList with the keypoints transformed into the target image transformation under the assumption that the keypoints are defined in
     * the source image transformation.
     *
     * If the target transformation has a different source coordinate system (eg. different camera socket) than the one the keypoints were originally generated
     * in, the remapping will be inaccurate due to the lack of depth information.
     *
     * @param source Source image transformation.
     * @param target Target image transformation.
     */
    KeypointsList transformTo(const ImgTransformation& source, const ImgTransformation& target) {
        KeypointsList transformedKeypoints = *this;
        for(auto& kp : transformedKeypoints.keypoints) {
            kp.transform(source, target);
        }
        return transformedKeypoints;
    }

    DEPTHAI_SERIALIZE(KeypointsList, keypoints, edges);
};

}  // namespace dai
