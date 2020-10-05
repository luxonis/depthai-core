#include "depthai/pipeline/AssetManager.hpp"

namespace dai
{
 
void AssetManager::add(Asset asset){
    // make sure that key doesn't exist already
    if(assetMap.count(asset.key) > 0) throw std::logic_error("An Asset with the key: " + asset.key + " already exists.");
    std::string key = asset.key;
    assetMap[key] = std::make_shared<Asset>(std::move(asset));
}

void AssetManager::add(const std::string& key, Asset asset){
    // Rename the asset with supplied key and store
    Asset a(key);
    a.data = std::move(asset.data);
    a.alignment = asset.alignment;
    add(std::move(a));
}

void AssetManager::set(const std::string& key, Asset asset){

    // Rename the asset with supplied key and store
    Asset a(key);
    a.data = std::move(asset.data);
    a.alignment = asset.alignment;
    assetMap[key] = std::make_shared<Asset>(std::move(a));
}

std::shared_ptr<Asset> AssetManager::get(const std::string& key){
    return assetMap[key];
}

std::vector<std::shared_ptr<Asset>> AssetManager::getAll(){
    std::vector<std::shared_ptr<Asset>> a;
    for(const auto& kv : assetMap){
        a.push_back(kv.second);
    }
    return a;
}

std::size_t AssetManager::size(){
    return assetMap.size();
}

void AssetManager::remove(const std::string& key){
    assetMap.erase(key);
}

void AssetManager::serialize(Assets& serAssets, std::vector<std::uint8_t>& assetStorage){
    using namespace std;
    vector<uint8_t> storage;

    for(auto& kv : assetMap){
        auto& a = *kv.second;

        // calculate additional bytes needed to offset to alignment
        int toAdd = 0;
        if(a.alignment > 1 && storage.size() % a.alignment != 0){
            toAdd = a.alignment - (storage.size() % a.alignment); 
        }

        // calculate offset
        std::uint32_t offset = storage.size() + toAdd;

        // Add alignment bytes
        storage.resize(storage.size() + toAdd);

        // copy data
        storage.insert(storage.end(), a.data.begin(), a.data.end());

        // Add to map the currently added asset
        AssetInternal internal = {};
        internal.offset = offset;
        internal.size = a.data.size();
        internal.alignment = a.alignment;
        map[a.key] = internal;

    }

    
    assetStorage = std::move(storage);
    serAssets = Assets(*this);
}

} // namespace dai
