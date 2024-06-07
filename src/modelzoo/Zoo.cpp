#include "depthai/modelzoo/Zoo.hpp"

#include <cpr/cpr.h>
#include <iostream>

namespace dai {

size_t ZooManager::computeModelHash() const {
    std::hash<std::string> hasher;
    return hasher(model.getName() + model.getVersion() + model.getPlatform());
}

std::string ZooManager::getCacheFolderName() const {
    size_t hash = computeModelHash();
    std::string hashstr = std::to_string(hash);
    return model.getName() + "_" + model.getVersion() + "_" + model.getPlatform() + "_" + hashstr;
}

std::string ZooManager::getCacheFolderPath(const std::string& cacheDir) const {
    return combinePaths(cacheDir, getCacheFolderName());
}

std::string ZooManager::combinePaths(const std::string& path1, const std::string& path2) const {
#if defined(_WIN32) && defined(_MSC_VER)
    const std::string separator = "\\";
#else
    const std::string separator = "/";
#endif
    return path1 + separator + path2;
}

void ZooManager::createCacheFolder() const {
    std::string cacheFolderName = getCacheFolderName();
    system(("mkdir -p " + cacheFolderName).c_str());
}

void ZooManager::removeCacheFolder() const {
    std::string cacheFolderName = getCacheFolderName();
    system(("rm -rf " + cacheFolderName).c_str());
}

bool ZooManager::isCached() const {
    return std::filesystem::exists(getCacheFolderName());
}

void ZooManager::downloadModel() {
    // Setup download url
    const std::string modelUrl = "http://mlcloud-services-load-balancer.default.stg.easyml/models/api/v1/models/download";
    cpr::Url url = cpr::Url{modelUrl};

    // Setup header
    cpr::Header header;
    header.insert({"x-team-id", "ecc1ccd9-daa6-45d0-a4c7-81fb11b4b0b9"});

    // Setup HTTP request parameters
    cpr::Parameters parameters;
    parameters.AddParameter(cpr::Parameter("model_slug", model.getName()));
    parameters.AddParameter(cpr::Parameter("platform", model.getPlatform()));

    // Send HTTP request
    cpr::Response response = cpr::Get(url, header, parameters);
    if(response.status_code != 200) {
        // Cleanup cache folder
        removeCacheFolder();

        throw std::runtime_error("Failed to download model");
    }

    // Unpack model download link - slice off first and last two characters
    // The returned text is in the format: ["link"]
    std::string link = response.text.substr(2, response.text.size() - 2 - 2);

    // Download model
    cpr::Url downloadUrl = cpr::Url{link};
    cpr::Response downloadResponse = cpr::Get(downloadUrl);

    // Save to tar file
    std::string folder = getCacheFolderName();
    std::string tarPath = combinePaths(folder, "model.tar");
    std::ofstream outStream(tarPath, std::ios::binary);
    outStream.write(downloadResponse.text.c_str(), downloadResponse.text.size());
    outStream.close();

    // Extract tar
    // TODO: This is not OS agnostic
    std::string command = "tar -xvf " + tarPath + " -C " + folder + " > /dev/null 2>&1";
    system(command.c_str());
}

void getModelFromZoo(const NNModelDescription& modelDescription, bool useCached) {
    // Initialize ZooManager
    ZooManager zooManager(modelDescription);

    // Check if model is cached
    bool modelIsCached = zooManager.isCached();
    bool useCachedModel = useCached && modelIsCached;

    // Use cached model if present and useCached is true
    if(useCachedModel) {
        // auto archive = zooManager.loadModelFromCache();
        // return archive;
        std::cout << "Using cached model" << std::endl;
        return;
    }

    // Remove cached model if present
    if(modelIsCached) {
        zooManager.removeCacheFolder();
    }

    // Create cache folder
    zooManager.createCacheFolder();

    // Download model
    zooManager.downloadModel();

    // Load model from cache
    // auto archive = zooManager.loadModelFromCache();
    // return archive;
    std::cout << "Using downloaded model" << std::endl;
    return;
}

}  // namespace dai