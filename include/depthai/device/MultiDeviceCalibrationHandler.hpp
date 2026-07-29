// IWYU pragma: private, include "depthai/depthai.hpp"
#pragma once

#include <array>
#include <cstdint>
#include <filesystem>
#include <map>
#include <optional>
#include <string>
#include <vector>

#include "depthai/common/CoordinateFrame.hpp"
#include "depthai/common/ImgTransformations.hpp"
#include "depthai/common/MultiDeviceCalibrationData.hpp"

namespace dai {

/**
 * Read/write interface to the rig calibration of a multi-device setup.
 *
 * The handler owns a forest of `RigEdge`s and resolves transformations between any two frames of the same connected
 * component in O(1). It deliberately holds no per-device calibration - intrinsics, distortion and intra-device
 * extrinsics always come from the live device.
 *
 * Frames in different connected components have no known relation, and querying across them throws.
 */
class MultiDeviceCalibrationHandler {
   public:
    MultiDeviceCalibrationHandler() = default;

    /**
     * Construct from rig data.
     *
     * @param data Rig edges and aliases
     * @param validate Validate that the graph is a forest (no cycles, no duplicate edges)
     */
    explicit MultiDeviceCalibrationHandler(MultiDeviceCalibrationData data, bool validate = true);

    /**
     * Construct from a rig JSON file, as written by `toJsonFile()`.
     *
     * @param jsonPath Path to the rig JSON file
     */
    explicit MultiDeviceCalibrationHandler(const std::filesystem::path& jsonPath);

    /**
     * Construct from rig JSON.
     */
    static MultiDeviceCalibrationHandler fromJson(const nlohmann::json& json, bool validate = true);

    /// Raw rig data.
    const MultiDeviceCalibrationData& getData() const;

    /// Rig data as JSON.
    nlohmann::json toJson() const;

    /**
     * Write the rig data to a JSON file.
     * @return true on success
     */
    bool toJsonFile(const std::filesystem::path& jsonPath) const;

    /// All device ids referenced by the rig, sorted and deduplicated.
    std::vector<std::string> getDeviceIds() const;

    /// All frames referenced by the rig, sorted.
    std::vector<CoordinateFrame> getFrames() const;

    /// True if the rig holds no edges.
    bool empty() const;

    /// True if a transformation between the two frames can be resolved.
    bool canTransform(const CoordinateFrame& from, const CoordinateFrame& to) const;

    /**
     * Deterministically elected root frame of the connected component the given frame belongs to. The root is the
     * lowest-ordered frame of the component, so it is stable as long as the component's frames do not change.
     *
     * @throws std::out_of_range if the frame is not part of the rig
     */
    CoordinateFrame getComponentRoot(const CoordinateFrame& frame) const;

    /// Frames of each connected component, in deterministic order (the first frame of each group is its root).
    std::vector<std::vector<CoordinateFrame>> getComponents() const;

    /**
     * Revision of the rig data, bumped on every modification. Consumers can use it to invalidate caches.
     */
    uint64_t getRevision() const;

    /**
     * Transformation from `from` to `to`, i.e. the matrix that maps a 3D point expressed in `from` into `to`.
     *
     * @param from Source frame
     * @param to Target frame
     * @param unit Length unit of the returned translation
     * @throws std::runtime_error if the frames are unknown or belong to different connected components
     */
    std::array<std::array<float, 4>, 4> getTransform(const CoordinateFrame& from, const CoordinateFrame& to, LengthUnit unit = LengthUnit::CENTIMETER) const;

    /**
     * Add an edge, replacing an existing edge between the same pair of frames (in either direction).
     * @throws std::runtime_error if the edge would create a cycle
     */
    void setEdge(const RigEdge& edge);

    /**
     * Remove the edge between the two frames, in either direction.
     * @return true if an edge was removed
     */
    bool removeEdge(const CoordinateFrame& from, const CoordinateFrame& to);

    /**
     * Resolve device aliases used in the rig data to actual device ids.
     *
     * Any frame whose `deviceId` matches a known alias is rewritten to the alias' target device id. Called by
     * `Pipeline::build()` for the pipeline's rig calibration.
     *
     * @param aliasToDeviceId Alias to device id mapping, merged on top of the mapping stored in the rig data
     */
    void resolveAliases(const std::map<std::string, std::string>& aliasToDeviceId = {});

    /**
     * Re-express extrinsics into a different reference frame, using the rig transformations.
     *
     * @param extrinsics Extrinsics to re-express, expressed w.r.t. a frame known to the rig
     * @param targetReference Reference frame to express the extrinsics in
     * @throws std::runtime_error if the current reference frame is unknown or unreachable from `targetReference`
     */
    void reexpress(Extrinsics& extrinsics, const CoordinateFrame& targetReference) const;

    /**
     * Re-express the extrinsics of an image transformation into a different reference frame.
     *
     * Only metadata is changed - the pixels and the intrinsics are untouched.
     */
    void reexpress(ImgTransformation& transformation, const CoordinateFrame& targetReference) const;

   private:
    /// Rebuild the derived per-frame transformations and validate the graph.
    void rebuild(bool validate);

    struct FrameInfo {
        /// Index of the connected component.
        size_t component = 0;
        /// Transformation from this frame to the component root.
        std::array<std::array<float, 4>, 4> toRoot{};
    };

    MultiDeviceCalibrationData data;
    std::map<CoordinateFrame, FrameInfo> frames;
    std::vector<CoordinateFrame> componentRoots;
    uint64_t revision = 0;
};

}  // namespace dai
