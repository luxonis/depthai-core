#pragma once

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <type_traits>
#include <vector>

#include "depthai/common/Point3f.hpp"
#include "depthai/common/Point3fRGBA.hpp"

namespace dai {
namespace node {
namespace internal {

/**
 * @internal
 * Voxel grid fusion of point clouds expressed in one coordinate system (PointCloudConfig::FusionMode::VOXEL).
 *
 * Every point is binned into a cubic voxel of the given edge length and every occupied voxel yields one output point:
 * the weighted centroid of the points inside it. A point weighs 1 / max(d, voxelSize)^2, where d is its distance to the
 * camera it was measured from; depth noise grows with the distance, so the closer camera dominates where cameras overlap.
 * Colors are averaged with the same weights. Voxels are emitted in the order their first point was seen, so the output is
 * deterministic for a given input.
 *
 * The scratch memory (hash table and accumulators) is kept between calls; one instance is meant to be reused frame after
 * frame from a single thread.
 */
class VoxelGridFusion {
   public:
    /// Points of one input cloud and the position of the camera they were measured from, both in the output coordinate system
    template <typename PointT>
    struct Source {
        const PointT* points = nullptr;
        size_t count = 0;
        Point3f cameraOrigin{};
    };

    /**
     * Fuse `sources` into `output` (resized to one point per occupied voxel).
     * @param voxelSize Voxel edge length in the unit of the points, must be positive
     * @returns Number of output points
     */
    template <typename PointT>
    size_t fuse(const std::vector<Source<PointT>>& sources, float voxelSize, std::vector<PointT>& output) {
        static_assert(std::is_same_v<PointT, Point3f> || std::is_same_v<PointT, Point3fRGBA>, "Unsupported point type");
        constexpr bool colored = std::is_same_v<PointT, Point3fRGBA>;

        size_t total = 0;
        for(const auto& source : sources) total += source.count;
        reset(total);

        const float inverseVoxel = 1.f / voxelSize;
        const float minimumDistanceSquared = voxelSize * voxelSize;
        for(const auto& source : sources) {
            const auto& origin = source.cameraOrigin;
            for(size_t i = 0; i < source.count; ++i) {
                const auto& p = source.points[i];
                const float dx = p.x - origin.x, dy = p.y - origin.y, dz = p.z - origin.z;
                const double weight = 1.0 / std::max(dx * dx + dy * dy + dz * dz, minimumDistanceSquared);
                auto& accumulator = accumulators[findOrInsert(voxelKey(p, inverseVoxel))];
                accumulator.x += weight * p.x;
                accumulator.y += weight * p.y;
                accumulator.z += weight * p.z;
                accumulator.weight += weight;
                if constexpr(colored) {
                    accumulator.r += weight * p.r;
                    accumulator.g += weight * p.g;
                    accumulator.b += weight * p.b;
                }
            }
        }

        output.resize(accumulators.size());
        for(size_t i = 0; i < accumulators.size(); ++i) {
            const auto& accumulator = accumulators[i];
            const double weight = accumulator.weight;
            auto& p = output[i];
            p.x = static_cast<float>(accumulator.x / weight);
            p.y = static_cast<float>(accumulator.y / weight);
            p.z = static_cast<float>(accumulator.z / weight);
            if constexpr(colored) {
                p.r = toColor(accumulator.r / weight);
                p.g = toColor(accumulator.g / weight);
                p.b = toColor(accumulator.b / weight);
                p.a = 255;
            }
        }
        return output.size();
    }

    /// Number of voxels of the last fuse() call
    size_t getVoxelCount() const {
        return accumulators.size();
    }

   private:
    // Double precision: a voxel can collect thousands of points whose weighted sums would otherwise lose the low bits
    struct Accumulator {
        double x = 0, y = 0, z = 0, weight = 0;
        double r = 0, g = 0, b = 0;
    };

    // Voxel coordinates are packed into 21 bits each (about +-1 million voxels per axis); bit 63 marks an occupied slot
    static constexpr int COORDINATE_BITS = 21;
    static constexpr int64_t COORDINATE_MIN = -(int64_t{1} << (COORDINATE_BITS - 1));
    static constexpr int64_t COORDINATE_MAX = (int64_t{1} << (COORDINATE_BITS - 1)) - 1;
    static constexpr uint64_t COORDINATE_MASK = (uint64_t{1} << COORDINATE_BITS) - 1;
    static constexpr uint64_t OCCUPIED = uint64_t{1} << 63;
    static constexpr uint64_t EMPTY_KEY = 0;
    static constexpr size_t MIN_CAPACITY = 1024;

    template <typename PointT>
    static uint64_t voxelKey(const PointT& p, float inverseVoxel) {
        auto quantize = [](float value) -> uint64_t {
            const auto index = static_cast<int64_t>(std::floor(value));
            return static_cast<uint64_t>(std::clamp(index, COORDINATE_MIN, COORDINATE_MAX)) & COORDINATE_MASK;
        };
        return OCCUPIED | quantize(p.x * inverseVoxel) | (quantize(p.y * inverseVoxel) << COORDINATE_BITS)
               | (quantize(p.z * inverseVoxel) << (2 * COORDINATE_BITS));
    }

    static size_t hash(uint64_t key) {
        // splitmix64 finalizer: spreads the packed coordinates over the table
        key ^= key >> 30;
        key *= 0xbf58476d1ce4e5b9ULL;
        key ^= key >> 27;
        key *= 0x94d049bb133111ebULL;
        key ^= key >> 31;
        return static_cast<size_t>(key);
    }

    static uint8_t toColor(double value) {
        return static_cast<uint8_t>(std::clamp(std::lround(value), 0L, 255L));
    }

    /// Clear the accumulators and size the table for `expectedPoints` input points (the table never shrinks)
    void reset(size_t expectedPoints) {
        accumulators.clear();
        lastKey = EMPTY_KEY;
        size_t capacity = MIN_CAPACITY;
        // Downsampling usually leaves far fewer voxels than points; the table grows when that does not hold
        while(capacity < expectedPoints / 2) capacity *= 2;
        capacity = std::max(capacity, keys.size());
        keys.assign(capacity, EMPTY_KEY);
        slotToVoxel.resize(capacity);
    }

    /// Index of the accumulator of `key`, inserting a new one when the voxel is seen for the first time
    uint32_t findOrInsert(uint64_t key) {
        if(key == lastKey) return lastVoxel;
        if((accumulators.size() + 1) * 10 > keys.size() * 7) grow();
        const size_t mask = keys.size() - 1;
        size_t slot = hash(key) & mask;
        while(keys[slot] != EMPTY_KEY && keys[slot] != key) slot = (slot + 1) & mask;
        if(keys[slot] == EMPTY_KEY) {
            keys[slot] = key;
            slotToVoxel[slot] = static_cast<uint32_t>(accumulators.size());
            accumulators.emplace_back();
        }
        lastKey = key;
        lastVoxel = slotToVoxel[slot];
        return lastVoxel;
    }

    void grow() {
        std::vector<uint64_t> oldKeys = std::move(keys);
        std::vector<uint32_t> oldSlots = std::move(slotToVoxel);
        const size_t capacity = oldKeys.size() * 2;
        keys.assign(capacity, EMPTY_KEY);
        slotToVoxel.resize(capacity);
        const size_t mask = capacity - 1;
        for(size_t i = 0; i < oldKeys.size(); ++i) {
            if(oldKeys[i] == EMPTY_KEY) continue;
            size_t slot = hash(oldKeys[i]) & mask;
            while(keys[slot] != EMPTY_KEY) slot = (slot + 1) & mask;
            keys[slot] = oldKeys[i];
            slotToVoxel[slot] = oldSlots[i];
        }
    }

    // Open-addressing hash table (linear probing): voxel key per slot and the index of its accumulator
    std::vector<uint64_t> keys;
    std::vector<uint32_t> slotToVoxel;
    // One accumulator per occupied voxel, in order of first appearance
    std::vector<Accumulator> accumulators;
    // Neighbouring pixels usually fall into the same voxel: the last lookup is cached
    uint64_t lastKey = EMPTY_KEY;
    uint32_t lastVoxel = 0;
};

}  // namespace internal
}  // namespace node
}  // namespace dai
