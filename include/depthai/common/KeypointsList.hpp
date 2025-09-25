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
        this->edges.clear();
        this->keypoints.clear();
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

// template <class T>
// struct KeypointsList {
//    private:
//     static_assert(std::is_base_of<Keypoint, T>::value, "T must be Keypoint or derived from Keypoint.");
//     std::vector<T> keypoints;
//     std::vector<Edge> edges;

//    public:
//     /** Default constructor. */
//     KeypointsList() = default;

//     /**
//      * Construct with keypoints and (optionally) edges.
//      * @param keypoints Vector of keypoints of type T.
//      * @param edges Vector of edge indices.
//      * @throws std::invalid_argument if any edge references an out-of-range index or creates a self-loop.
//      */
//     explicit KeypointsList(std::vector<T> keypoints, std::vector<Edge> edges = {}) : keypoints(std::move(keypoints)), edges(std::move(edges)) {
//         validateEdges();
//     }

//     /**
//      * Sets the keypoints list.
//      * @param kps list of keypoints of type T to set.
//      * @note This will clear any existing edges.
//      */
//     void setKeypoints(const std::vector<T>& kps) {
//         keypoints = kps;
//         edges.clear();
//     }

//     /**
//      * Sets the keypoints list (rvalue).
//      * @param kps list of keypoints of type T to set (moved).
//      * @note This will clear any existing edges.
//      */
//     void setKeypoints(std::vector<T>&& kps) {
//         keypoints = std::move(kps);
//         edges.clear();
//     }

//     /**
//      * Sets the keypoints list from 3D coordinates.
//      * Enabled only if T is constructible from Point3f.
//      * @param kps3 list of Point3f objects to set.
//      * @note This will clear any existing keypoints and edges.
//      */
//     template <typename U = T, typename = std::enable_if_t<std::is_constructible<U, Point3f>::value>>
//     void setKeypoints(const std::vector<Point3f>& kps3) {
//         edges.clear();
//         keypoints.clear();
//         keypoints.reserve(kps3.size());
//         for(const auto& p : kps3) keypoints.emplace_back(U{p});
//     }

//     /**
//      * Sets the keypoints list from 2D coordinates (z set to 0).
//      * Enabled only if T is constructible from Point2f.
//      * @param kps2 list of Point2f objects to set.
//      * @note This will clear any existing keypoints and edges.
//      */
//     template <typename U = T, typename = std::enable_if_t<std::is_constructible<U, Point2f>::value>, typename = void>
//     void setKeypoints(const std::vector<Point2f>& kps2) {
//         edges.clear();
//         keypoints.clear();
//         keypoints.reserve(kps2.size());
//         for(const auto& p : kps2) keypoints.emplace_back(U{p});
//     }

//     /**
//      * Sets the keypoints list and edges.
//      * @param keypoints list of keypoints of type T to set.
//      * @param edges list of edges to set.
//      * @note This will replace any existing keypoints and edges.
//      * @throws std::invalid_argument if any edge references an out-of-range index or creates a self-loop.
//      */
//     void setKeypoints(const std::vector<T>& keypoints, const std::vector<Edge>& edges) {
//         this->keypoints = keypoints;
//         this->edges = edges;
//         validateEdges();
//     }

//     /**
//      * Sets the keypoints list and edges (rvalue).
//      * @param keypoints list of keypoints of type T to set (moved).
//      * @param edges list of edges to set (moved).
//      * @note This will replace any existing keypoints and edges.
//      * @throws std::invalid_argument if any edge references an out-of-range index or creates a self-loop.
//      */
//     void setKeypoints(std::vector<T>&& keypoints, std::vector<Edge>&& edges) {
//         this->keypoints = std::move(keypoints);
//         this->edges = std::move(edges);
//         validateEdges();
//     }

//     /**
//      * Set the indices of the edges.
//      * @param edges Vector of edge indices.
//      * @throws std::invalid_argument if any edge references an out-of-range index or creates a self-loop.
//      */
//     void setEdges(const std::vector<Edge>& edges) {
//         this->edges = edges;
//         validateEdges();
//     }

//     /** Set the indices of the edges (rvalue). */
//     void setEdges(std::vector<Edge>&& edges) {
//         this->edges = std::move(edges);
//         validateEdges();
//     }

//     /**
//      * Get keypoints.
//      * @return Vector of keypoints of type T (by value; matches original API).
//      */
//     std::vector<T> getKeypoints() const {
//         return keypoints;
//     }

//     /**
//      * Get the indices of the edges.
//      * @return Vector of edge indices (by value; matches original API).
//      */
//     std::vector<Edge> getEdges() const {
//         return edges;
//     }

//     /**
//      * Get only coordinates of the keypoints (3D).
//      * @return Vector of Point3f coordinates.
//      */
//     std::vector<Point3f> getCoordinates3f() const {
//         std::vector<Point3f> coordinates;
//         coordinates.reserve(keypoints.size());
//         for(const auto& kp : keypoints) coordinates.emplace_back(kp.coordinates);
//         return coordinates;
//     }

//     /**
//      * Get only 2D coordinates of the keypoints and drop the z axis values.
//      * @return Vector of Point2f coordinates.
//      */
//     std::vector<Point2f> getCoordinates2f() const {
//         std::vector<Point2f> coordinates;
//         coordinates.reserve(keypoints.size());
//         for(const auto& kp : keypoints) coordinates.emplace_back(Point2f{kp.coordinates.x, kp.coordinates.y});
//         return coordinates;
//     }

//     /**
//      * Get keypoint labels.
//      * @return Vector of keypoint label names.
//      */
//     std::vector<std::string> getLabels() const {
//         std::vector<std::string> labels;
//         labels.reserve(keypoints.size());
//         for(const auto& kp : keypoints) labels.push_back(kp.labelName);
//         return labels;
//     }

//     /**
//      * Validate that all edges reference valid, distinct keypoint indices.
//      * @throws std::invalid_argument if an edge references an out-of-range index or creates a self-loop.
//      */
//     void validateEdges() const {
//         const auto n = keypoints.size();
//         for(const auto& e : edges) {
//             if(e[0] >= n || e[1] >= n) throw std::invalid_argument("Edge index out of range.");
//             if(e[0] == e[1]) throw std::invalid_argument("Self-loop edges are not allowed.");
//         }
//     }

//     // DEPTHAI_SERIALIZE(KeypointsList<T>, keypoints, edges); // templates usually need explicit specializations
// };

}  // namespace dai
