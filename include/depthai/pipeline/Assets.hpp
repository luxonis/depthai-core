#pragma once

// standard
#include <cstdint>
#include <exception>
#include <unordered_map>

// project
#include "depthai/utility/Serialization.hpp"

namespace dai {

// This class represent a single asset
struct AssetView {
    std::uint8_t* data;
    std::uint32_t size;
    std::uint32_t alignment = 1;
    /**
     * Construct a view into asset data.
     */
    AssetView(std::uint8_t* d, std::uint32_t s, std::uint32_t a = 1) : data(d), size(s), alignment(a) {}
};

// This is a serializable class, which acts as readonly access to assets
class Assets {
   protected:
    std::uint8_t* pStorageStart = nullptr;

    struct AssetInternal {
        std::uint32_t offset, size, alignment;
        DEPTHAI_SERIALIZE(AssetInternal, offset, size, alignment);
    };

    // maps string to Asset
    std::unordered_map<std::string, AssetInternal> map;

   public:
    /**
     * Set the backing storage for assets.
     */
    void setStorage(std::uint8_t* ps) {
        pStorageStart = ps;
    }

    /**
     * Return true if an asset key exists.
     */
    bool has(const std::string& key) {
        return (map.count(key) > 0);
    }

    /**
     * Get an asset view by key.
     */
    AssetView get(const std::string& key) {
        AssetInternal internal = map.at(key);
        return {pStorageStart + internal.offset, internal.size, internal.alignment};
    }

    /**
     * Get all assets as key/view pairs.
     */
    std::vector<std::pair<std::string, AssetView>> getAll() {
        std::vector<std::pair<std::string, AssetView>> allAssets;
        for(const auto& kv : map) {
            allAssets.emplace_back(kv.first, get(kv.first));
        }
        return allAssets;
    }

    DEPTHAI_SERIALIZE(Assets, map);
};

}  // namespace dai
