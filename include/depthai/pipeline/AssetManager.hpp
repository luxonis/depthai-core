#pragma once

#include <filesystem>
#include <map>
#include <memory>
#include <mutex>
#include <vector>

#include "depthai/pipeline/Assets.hpp"

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
    std::vector<std::uint8_t>& getData();
    const std::vector<std::uint8_t>& getData() const;
    void setData(std::vector<std::uint8_t> data);
    std::size_t getSize() const;
    std::string getRelativeUri();

    /// Set the backing file and its expected size. The file must remain available and unchanged until the asset is materialized or serialized.
    void setFile(std::filesystem::path path, std::size_t size);

   private:
    friend class AssetManager;

    mutable std::shared_ptr<std::mutex> dataMutex = std::make_shared<std::mutex>();
    std::filesystem::path path;
    std::size_t size = 0;
    mutable std::vector<std::uint8_t> lazyData;
    mutable bool dataLoaded = false;

    void loadData() const;
};

class AssetsMutable : public Assets {
   public:
    void set(const std::string&, std::uint32_t offset, std::uint32_t size, std::uint32_t alignment);
};

// Subclass which has its own storage
/**
 * @brief AssetManager can store assets and serialize
 */
class AssetManager /*: public Assets*/ {
    std::map<std::string, std::shared_ptr<Asset>> assetMap;
    std::string rootPath;

    std::string getRelativeKey(const std::string& key) const;

   public:
    AssetManager();
    AssetManager(const std::string& rootPath);
    /**
     * Adds all assets in an array to the AssetManager
     * @param assets Vector of assets to add
     */
    void addExisting(const std::vector<std::shared_ptr<Asset>>& assets);

    /**
     * Get root path of the asset manager
     * @returns Root path
     */
    std::string getRootPath();

    /**
     * Set root path of the asset manager
     * @param rootPath Root path
     */
    void setRootPath(const std::string& rootPath);

    /**
     * Adds or overwrites an asset object to AssetManager.
     * @param asset Asset to add
     * @returns Shared pointer to asset
     */
    std::shared_ptr<dai::Asset> set(Asset asset);

    /**
     * Adds or overwrites an asset object to AssetManager with a specified key.
     * Key value will be assigned to an Asset as well
     *
     * @param key Key under which the asset should be stored
     * @param asset Asset to store
     * @returns Shared pointer to asset
     */
    std::shared_ptr<dai::Asset> set(const std::string& key, Asset asset);

    /**
     * Loads a file into the asset manager under the specified key.
     *
     * @param key Key under which the asset should be stored
     * @param path Path to file which to load as asset
     * @param alignment [Optional] alignment of asset data in asset storage. Default is 64B
     */
    std::shared_ptr<dai::Asset> set(const std::string& key, const std::filesystem::path& path, int alignment = 64);

    /**
     * Registers a file-backed asset under the specified key. Its contents are loaded lazily, so the file must remain available and unchanged
     * until the pipeline is serialized or Asset::getData() is called.
     *
     * @param key Key under which the asset should be stored
     * @param path Path to the file backing the asset
     * @param alignment [Optional] alignment of asset data in asset storage. Default is 64B
     */
    std::shared_ptr<dai::Asset> setLazy(const std::string& key, const std::filesystem::path& path, int alignment = 64);

    /**
     * Loads data into the asset manager under the specified key.
     *
     * @param key Key under which the asset should be stored
     * @param data Asset data
     * @param alignment [Optional] alignment of asset data in asset storage. Default is 64B
     * @returns Shared pointer to asset
     */
    std::shared_ptr<dai::Asset> set(const std::string& key, const std::vector<std::uint8_t>& data, int alignment = 64);
    std::shared_ptr<dai::Asset> set(const std::string& key, std::vector<std::uint8_t>&& data, int alignment = 64);

    /**
     * @returns Asset assigned to the specified key or a nullptr otherwise
     */
    std::shared_ptr<const Asset> get(const std::string& key) const;

    /**
     * @returns Asset assigned to the specified key or a nullptr otherwise
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
    void serialize(AssetsMutable& assets, std::vector<std::uint8_t>& assetStorage, std::string prefix = "") const;
    /// Calculates the size of the serialized data
    std::size_t getSerializedSize(std::size_t offset = 0) const;
};

}  // namespace dai
