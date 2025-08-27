#pragma once

// std
#include <array>
#include <cstdint>
#include <stdexcept>
#include <string>

// project
#include "depthai/common/Keypoint.hpp"
#include "depthai/common/Point2f.hpp"
#include "depthai/common/Point3f.hpp"
#include "depthai/utility/Serialization.hpp"

namespace dai {
using Edge = std::array<uint32_t, 2>;

struct KeypointsList {
   private:
    std::vector<Keypoint> keypoints;
    std::vector<Edge> edges;

   public:
    KeypointsList() = default;
    explicit KeypointsList(std::vector<Keypoint> keypoints, std::vector<Edge> edges) : keypoints(std::move(keypoints)), edges(std::move(edges)) {
        validateEdges();
    }
    explicit KeypointsList(std::vector<Keypoint> keypoints) : keypoints(std::move(keypoints)) {}

    /**
     * Sets the keypoints list.
     * @param keypoints list of Keypoint objects to set.
     * @note This will clear any existing keypoints and edges.
     */
    void setKeypoints(const std::vector<Keypoint> kps) {
        keypoints = kps;
        edges.clear();
    }

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

    /**
     * Sets the keypoints list.
     * @param keypoints list of Keypoint objects and edges to set.
     * @note This will clear any existing keypoints and edges.
     */
    void setKeypoints(const std::vector<Keypoint> keypoints, const std::vector<Edge> edges) {
        this->keypoints = std::move(keypoints);
        this->edges = std::move(edges);
        validateEdges();
    }

    /**
     * Set the indices of the edges.
     * @param edges Vector of edge indices.
     */

    void setEdges(const std::vector<Edge> edges) {
        this->edges = edges;
        validateEdges();
    }

    /**
     * Get keypoints.
     * @return Vector of Keypoint objects.
     */
    std::vector<Keypoint> getKeypoints() const {
        return keypoints;
    }

    /**
     * Get the indices of the edges.
     * @return Vector of edge indices.
     */
    std::vector<Edge> getEdges() const {
        return edges;
    }

    /**
     * Get only coordinates of the keypoints.
     * @return Vector of Point3f coordinates.
     */
    std::vector<Point3f> getCoordinates3f() const {
        std::vector<Point3f> coordinates;

        for(const auto& kp : keypoints) {
            coordinates.emplace_back(kp.coordinates);
        }
        return coordinates;
    }

    /**
     * Get only 2D coordinates of the keypoints and drop the z axis values.
     * @return Vector of Point2f coordinates.
     */
    std::vector<Point2f> getCoordinates2f() const {
        std::vector<Point2f> coordinates;

        for(const auto& kp : keypoints) {
            coordinates.emplace_back(Point2f(kp.coordinates.x, kp.coordinates.y));
        }
        return coordinates;
    }

    /**
     * Get keypoint labels.
     * @return Vector of keypoint label names.
     */
    std::vector<std::string> getLabels() const {
        std::vector<std::string> labels;
        labels.reserve(keypoints.size());
        for(const auto& kp : keypoints) {
            labels.push_back(kp.labelName);
        }
        return labels;
    }

    void validateEdges() {
        const auto n = keypoints.size();

        for(auto& e : edges) {
            if(e[0] >= n || e[1] >= n) {
                throw std::invalid_argument("Edge index out of range.");
            }
            if(e[0] == e[1]) {
                throw std::invalid_argument("Self-loop edges are not allowed.");
            }
        }
    }

    DEPTHAI_SERIALIZE(KeypointsList, keypoints, edges);
};

}  // namespace dai
