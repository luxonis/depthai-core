#pragma once

#include <map>
#include <memory>
#include <vector>

#include "depthai-shared/pipeline/Assets.hpp"

namespace dai {

/**
 * @brief Asset is identified with string key and can store arbitrary binary data
 */
struct Asset {
    Asset() = default;
    explicit Asset(std::string k) : key(std::move(k)) {}
    const std::string key;
    std::vector<std::uint8_t> data;
    std::uint32_t alignment = 1;
};

class AssetsMutable : public Assets {
   public:
    void set(std::string, std::uint32_t offset, std::uint32_t size, std::uint32_t alignment);
};

// Subclass which has its own storage
/**
 * @brief AssetManager can store assets and serialize
 */
class AssetManager /*: public Assets*/ {
    std::map<std::string, std::shared_ptr<Asset>> assetMap;

   public:
    /**
     * Adds all assets in an array to the AssetManager
     * @param assets Vector of assets to add
     */
    void addExisting(std::vector<std::shared_ptr<Asset>> assets);

    /**
     * Adds an asset object to AssetManager.
     * @param asset Asset to add
     */
    void add(Asset asset);

    /**
     * Adds an asset object to AssetManager with a specificied key.
     * Key value will be assigned to an Asset as well
     *
     * If asset with key already exists, the function throws an error
     *
     * @param key Key under which the asset should be stored
     * @param asset Asset to store
     */
    void add(const std::string& key, Asset asset);

    /**
     * Adds or overwrites existing asset with a specificied key.
     *
     * @param key Key under which the asset should be stored
     * @param asset Asset to store
     */
    void set(const std::string& key, Asset asset);

    /**
     * @returns Asset assigned to the specified key or throws an error otherwise
     */
    std::shared_ptr<const Asset> get(const std::string& key) const;

    /**
     * @returns Asset assigned to the specified key or throws an error otherwise
     */
    std::shared_ptr<Asset> get(const std::string& key);

    /**
     * @returns All asset stored in the AssetManager
     */
    std::vector<std::shared_ptr<const Asset>> getAll() const;

    /**
     * @returns All asset stored in the AssetManager
     */
    std::vector<std::shared_ptr<Asset>> getAll();

    /**
     * @returns Number of asset stored in the AssetManager
     */
    std::size_t size() const;

    /**
     * Removes asset with key
     * @param key Key of asset to remove
     */
    void remove(const std::string& key);

    /// Serializes
    void serialize(Assets& serAssets, std::vector<std::uint8_t>& assetStorage) const;
};

}  // namespace dai
