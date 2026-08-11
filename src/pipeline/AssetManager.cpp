#include "depthai/pipeline/AssetManager.hpp"

#include <fmt/std.h>

#include "spdlog/fmt/fmt.h"
#include "utility/spdlog-fmt.hpp"

// std
#include <algorithm>
#include <array>
#include <fstream>
#include <limits>

namespace dai {

namespace {

constexpr std::size_t MAX_ASSET_STORAGE_SIZE = std::numeric_limits<std::uint32_t>::max();

std::size_t getSerializedEndOffset(std::size_t offset, std::uint32_t alignment, std::size_t assetSize) {
    if(alignment == 0) {
        throw std::runtime_error("Asset alignment cannot be zero");
    }

    if(offset > MAX_ASSET_STORAGE_SIZE || assetSize > MAX_ASSET_STORAGE_SIZE) {
        throw std::runtime_error("Asset storage cannot exceed 4 GiB");
    }

    std::size_t padding = 0;
    if(alignment > 1 && offset % alignment != 0) {
        padding = alignment - (offset % alignment);
    }

    if(padding > MAX_ASSET_STORAGE_SIZE - offset || assetSize > MAX_ASSET_STORAGE_SIZE - offset - padding) {
        throw std::runtime_error("Asset storage cannot exceed 4 GiB");
    }
    return offset + padding + assetSize;
}

}  // namespace

std::string Asset::getRelativeUri() {
    return fmt::format("{}:{}", "asset", key);
}

std::vector<std::uint8_t>& Asset::getData() {
    if(data.empty() && !dataLoaded && !path.empty()) {
        std::ifstream stream(path, std::ios::in | std::ios::binary);
        if(!stream.is_open()) {
            throw std::runtime_error(fmt::format("Cannot load asset, file at path {} doesn't exist.", path));
        }

        auto loadedData = std::vector<std::uint8_t>(size);
        std::size_t loadedSize = 0;
        while(loadedSize < size) {
            const auto bytesToRead = std::min<std::size_t>(size - loadedSize, 1024 * 1024);
            stream.read(reinterpret_cast<char*>(loadedData.data() + loadedSize), bytesToRead);
            const auto bytesRead = stream.gcount();
            if(bytesRead != static_cast<std::streamsize>(bytesToRead)) {
                throw std::runtime_error(fmt::format("Cannot load asset, file at path {} has changed size.", path));
            }
            loadedSize += static_cast<std::size_t>(bytesRead);
        }
        if(stream.peek() != std::char_traits<char>::eof()) {
            throw std::runtime_error(fmt::format("Cannot load asset, file at path {} has changed size.", path));
        }

        data = std::move(loadedData);
        dataLoaded = true;
    }
    return data;
}

std::size_t Asset::getSize() const {
    return path.empty() ? data.size() : size;
}

AssetManager::AssetManager() {}
AssetManager::AssetManager(const std::string& rootPath) : rootPath{rootPath} {}

std::string AssetManager::getRootPath() {
    return rootPath;
}

void AssetManager::setRootPath(const std::string& rootPath) {
    this->rootPath = rootPath;
}

std::string AssetManager::getRelativeKey(const std::string& key) const {
    // Check if asset key is absolute or relative
    std::string relativeKey = "";
    if(key.size() == 0) {
        return relativeKey;
    }

    if(key[0] == '/') {                // Absolute path
        if(key.find(rootPath) == 0) {  // Root path of the node is contained in the key
            int rootPathLen = rootPath.size();
            relativeKey = key.substr(rootPathLen);
        } else {
            return "";
        }
    } else {  // Relative path
        relativeKey = key;
    }

    return relativeKey;
}

std::shared_ptr<dai::Asset> AssetManager::set(Asset asset) {
    std::string key = asset.key;
    assetMap[key] = std::make_shared<Asset>(std::move(asset));
    return assetMap[key];
}

std::shared_ptr<dai::Asset> AssetManager::set(const std::string& key, Asset asset) {
    // Rename the asset with supplied key and store
    Asset a(key);
    a.data = std::move(asset.data);
    a.path = std::move(asset.path);
    a.size = a.path.empty() ? 0 : asset.size;
    a.dataLoaded = asset.dataLoaded;
    a.alignment = asset.alignment;
    return set(std::move(a));
}

std::shared_ptr<dai::Asset> AssetManager::set(const std::string& key, const std::filesystem::path& path, int alignment) {
    // Load binary file at path
    std::ifstream stream(path, std::ios::in | std::ios::binary);
    if(!stream.is_open()) {
        // Throw an error
        // TODO(themarpe) - Unify exceptions into meaningful groups
        throw std::runtime_error(fmt::format("Cannot load asset, file at path {} doesn't exist.", path));
    }

    // Create an asset
    Asset binaryAsset(key);
    binaryAsset.alignment = alignment;
    binaryAsset.path = path;
    binaryAsset.size = static_cast<std::size_t>(std::filesystem::file_size(path));
    // Store asset
    return set(std::move(binaryAsset));
}

std::shared_ptr<dai::Asset> AssetManager::set(const std::string& key, const std::vector<std::uint8_t>& data, int alignment) {
    // Create an asset
    Asset binaryAsset(key);
    binaryAsset.alignment = alignment;
    binaryAsset.data = data;
    // Store asset
    return set(std::move(binaryAsset));
}

std::shared_ptr<dai::Asset> AssetManager::set(const std::string& key, std::vector<std::uint8_t>&& data, int alignment) {
    // Create an asset
    Asset binaryAsset(key);
    binaryAsset.alignment = alignment;
    binaryAsset.data = std::move(data);
    // Store asset
    return set(std::move(binaryAsset));
}

std::shared_ptr<const Asset> AssetManager::get(const std::string& key) const {
    std::string relativeKey = getRelativeKey(key);
    if(assetMap.count(relativeKey) == 0) {
        return nullptr;
    }
    return assetMap.at(relativeKey);
}

std::shared_ptr<Asset> AssetManager::get(const std::string& key) {
    std::string relativeKey = getRelativeKey(key);
    if(assetMap.count(relativeKey) == 0) {
        return nullptr;
    }
    return assetMap.at(relativeKey);
}

void AssetManager::addExisting(const std::vector<std::shared_ptr<Asset>>& assets) {
    // make sure that key doesn't exist already
    for(const auto& asset : assets) {
        if(assetMap.count(asset->key) > 0) throw std::logic_error("An Asset with the key: " + asset->key + " already exists.");
        std::string key = asset->key;
        assetMap[key] = asset;
    }
}

std::vector<std::shared_ptr<const Asset>> AssetManager::getAll() const {
    std::vector<std::shared_ptr<const Asset>> a;
    for(const auto& kv : assetMap) {
        a.push_back(kv.second);
    }
    return a;
}

std::vector<std::shared_ptr<Asset>> AssetManager::getAll() {
    std::vector<std::shared_ptr<Asset>> a;
    for(const auto& kv : assetMap) {
        a.push_back(kv.second);
    }
    return a;
}

std::size_t AssetManager::size() const {
    return assetMap.size();
}

void AssetManager::remove(const std::string& key) {
    assetMap.erase(key);
}

void AssetManager::serialize(AssetsMutable& mutableAssets, std::vector<std::uint8_t>& storage, std::string prefix) const {
    using namespace std;

    if(prefix.empty()) {
        prefix = rootPath;
    }

    const auto storageStart = storage.size();
    const auto mutableAssetsStart = mutableAssets;
    try {
        for(auto& kv : assetMap) {
            auto& a = *kv.second;

            const auto assetSize = a.getSize();
            const auto assetStorageStart = storage.size();

            // Calculate additional bytes needed to offset to alignment.
            std::size_t toAdd = 0;
            if(a.alignment > 1 && storage.size() % a.alignment != 0) {
                toAdd = a.alignment - (storage.size() % a.alignment);
            }

            const auto storageEnd = getSerializedEndOffset(storage.size(), a.alignment, assetSize);
            storage.reserve(storageEnd);

            // calculate offset
            std::uint32_t offset = static_cast<uint32_t>(storage.size()) + toAdd;

            // Add alignment bytes
            storage.resize(storage.size() + toAdd);

            if(!a.path.empty()) {
                try {
                    std::ifstream stream(a.path, std::ios::in | std::ios::binary);
                    if(!stream.is_open()) {
                        throw std::runtime_error(fmt::format("Cannot load asset, file at path {} doesn't exist.", a.path));
                    }
                    std::vector<std::uint8_t> buffer(1024 * 1024);
                    std::size_t streamedSize = 0;
                    while(streamedSize < assetSize) {
                        const auto bytesToRead = std::min(buffer.size(), assetSize - streamedSize);
                        stream.read(reinterpret_cast<char*>(buffer.data()), bytesToRead);
                        auto bytesRead = stream.gcount();
                        if(bytesRead != static_cast<std::streamsize>(bytesToRead)) {
                            throw std::runtime_error(fmt::format("Asset at path {} changed while serializing.", a.path));
                        }
                        storage.insert(storage.end(), buffer.data(), buffer.data() + bytesRead);
                        streamedSize += static_cast<std::size_t>(bytesRead);
                    }
                    if(stream.peek() != std::char_traits<char>::eof()) {
                        throw std::runtime_error(fmt::format("Asset at path {} changed while serializing.", a.path));
                    }
                } catch(...) {
                    storage.resize(assetStorageStart);
                    throw;
                }
            } else {
                storage.insert(storage.end(), a.data.begin(), a.data.end());
            }

            // Add to map the currently added asset
            mutableAssets.set(prefix + a.key, offset, static_cast<uint32_t>(assetSize), a.alignment);
        }
    } catch(...) {
        storage.resize(storageStart);
        mutableAssets = mutableAssetsStart;
        throw;
    }
}

std::size_t AssetManager::getSerializedSize(std::size_t offset) const {
    for(const auto& kv : assetMap) {
        const auto& a = *kv.second;
        offset = getSerializedEndOffset(offset, a.alignment, a.getSize());
    }
    return offset;
}

void AssetsMutable::set(const std::string& key, std::uint32_t offset, std::uint32_t size, std::uint32_t alignment) {
    AssetInternal internal = {};
    internal.offset = offset;
    internal.size = size;
    internal.alignment = alignment;
    map[key] = internal;
}

}  // namespace dai
