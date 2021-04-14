#include "depthai/pipeline/AssetManager.hpp"

namespace dai {

void AssetManager::add(Asset asset) {
    // make sure that key doesn't exist already
    if(assetMap.count(asset.key) > 0) throw std::logic_error("An Asset with the key: " + asset.key + " already exists.");
    std::string key = asset.key;
    assetMap[key] = std::make_shared<Asset>(std::move(asset));
}

void AssetManager::add(const std::string& key, Asset asset) {
    // Rename the asset with supplied key and store
    Asset a(key);
    a.data = std::move(asset.data);
    a.alignment = asset.alignment;
    add(std::move(a));
}

void AssetManager::set(const std::string& key, Asset asset) {
    // Rename the asset with supplied key and store
    Asset a(key);
    a.data = std::move(asset.data);
    a.alignment = asset.alignment;
    assetMap[key] = std::make_shared<Asset>(std::move(a));
}

std::shared_ptr<const Asset> AssetManager::get(const std::string& key) const {
    return assetMap.at(key);
}

std::shared_ptr<Asset> AssetManager::get(const std::string& key) {
    return assetMap[key];
}

void AssetManager::addExisting(std::vector<std::shared_ptr<Asset>> assets) {
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

void AssetManager::serialize(Assets& serAssets, std::vector<std::uint8_t>& assetStorage) const {
    using namespace std;
    vector<uint8_t> storage;
    AssetsMutable mutableAssets;

    for(auto& kv : assetMap) {
        auto& a = *kv.second;

        // calculate additional bytes needed to offset to alignment
        int toAdd = 0;
        if(a.alignment > 1 && storage.size() % a.alignment != 0) {
            toAdd = a.alignment - (storage.size() % a.alignment);
        }

        // calculate offset
        std::uint32_t offset = static_cast<uint32_t>(storage.size()) + toAdd;

        // Add alignment bytes
        storage.resize(storage.size() + toAdd);

        // copy data
        storage.insert(storage.end(), a.data.begin(), a.data.end());

        // Add to map the currently added asset
        mutableAssets.set(a.key, offset, static_cast<uint32_t>(a.data.size()), a.alignment);
    }

    assetStorage = std::move(storage);
    serAssets = Assets(mutableAssets);
}

void AssetsMutable::set(std::string key, std::uint32_t offset, std::uint32_t size, std::uint32_t alignment) {
    AssetInternal internal = {};
    internal.offset = offset;
    internal.size = size;
    internal.alignment = alignment;
    map[key] = internal;
}

}  // namespace dai
