#pragma once

#include <array>
#include <cstdint>
#include <stdexcept>

#include "depthai/common/Point2f.hpp"
#include "depthai/common/Point3f.hpp"

namespace dai {
using Edge = std::array<uint32_t, 2>;

template <class KeypointT>
struct KeypointsListT {
   protected:
    std::vector<KeypointT> keypoints;
    std::vector<Edge> edges;

   public:
    KeypointsListT() = default;
    explicit KeypointsListT(std::vector<KeypointT> keypoints, std::vector<Edge> edges) : keypoints(std::move(keypoints)), edges(std::move(edges)) {
        validateEdges();
    }
    explicit KeypointsListT(std::vector<KeypointT> keypoints) : keypoints(std::move(keypoints)) {}
    ~KeypointsListT() = default;

    /*
     * Iterator support
     */

    using value_type = KeypointT;
    using iterator = typename std::vector<KeypointT>::iterator;
    using const_iterator = typename std::vector<KeypointT>::const_iterator;

    iterator begin() noexcept {
        return keypoints.begin();
    }
    iterator end() noexcept {
        return keypoints.end();
    }

    const_iterator begin() const noexcept {
        return keypoints.begin();
    }
    const_iterator end() const noexcept {
        return keypoints.end();
    }
    const_iterator cbegin() const noexcept {
        return keypoints.cbegin();
    }
    const_iterator cend() const noexcept {
        return keypoints.cend();
    }
    bool empty() const noexcept {
        return keypoints.empty();
    }
    size_t size() const noexcept {
        return keypoints.size();
    }
    value_type& operator[](size_t i) {
        return keypoints[i];
    }
    const value_type& operator[](size_t i) const {
        return keypoints[i];
    }

    /*
     * Common API
     */

    /**
     * Sets the keypoints list.
     * @param keypoints list of Keypoint objects to set.
     * @note This will clear any existing keypoints and edges.
     */
    void setKeypoints(const std::vector<KeypointT> kps) {
        keypoints = kps;
        edges.clear();
    }

    /**
     * Sets the keypoints list.
     * @param keypoints list of Keypoint objects and edges to set.
     * @note This will clear any existing keypoints and edges.
     */
    void setKeypoints(std::vector<KeypointT> keypoints, std::vector<Edge> edges) {
        this->keypoints = std::move(keypoints);
        this->edges = std::move(edges);
        validateEdges();
    }

    /**
     * Set the indices of the edges.
     * @param edges Vector of edge indices.
     */

    void setEdges(std::vector<Edge> edges) {
        this->edges = std::move(edges);
        validateEdges();
    }

    /**
     * Get keypoints.
     * @return Vector of Keypoint objects.
     */
    std::vector<KeypointT> getKeypoints() const {
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
     * Get only image coordinates of the keypoints.
     * @return Vector of Point3f coordinates.
     */
    std::vector<Point3f> getPoints3f() const {
        std::vector<Point3f> coordinates;
        coordinates.reserve(keypoints.size());

        for(const auto& kp : keypoints) {
            coordinates.emplace_back(kp.imageCoordinates);
        }
        return coordinates;
    }

    /**
     * Get only image 2D coordinates of the keypoints and drop the z axis values.
     * @return Vector of Point2f coordinates.
     */
    std::vector<Point2f> getPoints2f() const {
        std::vector<Point2f> coordinates;
        coordinates.reserve(keypoints.size());

        for(const auto& kp : keypoints) {
            coordinates.emplace_back(Point2f(kp.imageCoordinates.x, kp.imageCoordinates.y));
        }
        return coordinates;
    }

   private:
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
};

}  // namespace dai
