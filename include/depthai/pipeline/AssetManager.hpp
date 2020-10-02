#pragma once

#include <map>
#include <vector>
#include <memory>

#include "depthai-shared/Assets.hpp"

namespace dai
{
    

struct Asset{
    Asset() = default;
    explicit Asset(std::string k) : key(std::move(k)){} 
    const std::string key;
    std::vector<std::uint8_t> data;
    std::uint32_t alignment = 1;
};

// Subclass which has its own storage
class AssetManager : public Assets {
    std::map<std::string, std::shared_ptr<Asset>> assetMap;

public:

    void add(const Asset& asset);
    void add(std::string key, const Asset& asset);
    void set(std::string key, const Asset& asset);
    std::shared_ptr<Asset> get(std::string key);
    std::vector<std::shared_ptr<Asset>> getAll();
    std::size_t size();
    void remove(std::string key);
    void serialize(Assets& serAssets, std::vector<std::uint8_t>& assetStorage);

};




} // namespace dai

