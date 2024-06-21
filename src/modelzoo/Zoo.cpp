#include "depthai/modelzoo/Zoo.hpp"

#include <cpr/cpr.h>

#include <depthai/utility/Path.hpp>
#include <iostream>

#include "../utility/sha1.hpp"

namespace dai {

std::string ZooManager::computeModelHash() const {
    SHA1 hasher;
    hasher.update(modelDescription.getModelSlug() + modelDescription.getmodelVersionSlug() + modelDescription.getPlatform());
    return hasher.final();
}

std::string ZooManager::getModelCacheFolderName() const {
    const std::string separator = "--";
    std::string foldername = "";

    // REQUIRED fields first
    foldername += "model_slug=" + modelDescription.getModelSlug() + separator;
    foldername += "platform=" + modelDescription.getPlatform() + separator;

    // OPTIONAL fields
    if(modelDescription.getmodelVersionSlug().size() > 0) foldername += "model_version_slug=" + modelDescription.getmodelVersionSlug() + separator;

    // Append hash
    foldername += "hash=" + computeModelHash();

    return foldername;
}

std::string ZooManager::getModelCacheFolderPath(const std::string& cacheDirectory) const {
    return combinePaths(cacheDirectory, getModelCacheFolderName());
}

std::string ZooManager::combinePaths(const std::string& path1, const std::string& path2) const {
    return std::filesystem::path(path1).append(path2).string();
}

void ZooManager::createCacheFolder() const {
    std::string cacheFolderName = getModelCacheFolderPath(cacheDirectory);
    std::filesystem::create_directories(cacheFolderName);
}

void ZooManager::removeModelCacheFolder() const {
    std::string cacheFolderName = getModelCacheFolderPath(cacheDirectory);
    std::filesystem::remove_all(cacheFolderName);
}

bool ZooManager::isCached() const {
    return checkExists(getModelCacheFolderPath(cacheDirectory));
}

bool ZooManager::checkExists(const std::string& path) const {
    return std::filesystem::exists(path);
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
    parameters.AddParameter(cpr::Parameter("model_slug", modelDescription.getModelSlug()));
    parameters.AddParameter(cpr::Parameter("platform", modelDescription.getPlatform()));

    // Add optional parameters
    std::string modelVersionSlug = modelDescription.getmodelVersionSlug();
    if(modelVersionSlug.size() > 0) {
        parameters.AddParameter(cpr::Parameter("model_version_slug", modelVersionSlug));
    }

    // Send HTTP request
    cpr::Response response = cpr::Get(url, header, parameters);
    if(response.status_code != cpr::status::HTTP_OK) {
        // Cleanup cache folder
        removeModelCacheFolder();

        // Inform the user about the error and print out model description
        // for easier debugging
        throw std::runtime_error("Failed to download model\n" + modelDescription.toString() + "\nError: " + response.text
                                 + "\n HTTP status code: " + std::to_string(response.status_code));
    }

    // Unpack model download link - slice off first and last two characters
    // The returned text is in the format: ["link"]
    std::string link = response.text.substr(2, response.text.size() - 2 - 2);

    // Download model
    cpr::Url downloadUrl = cpr::Url{link};
    cpr::Response downloadResponse = cpr::Get(downloadUrl);
    if(downloadResponse.status_code != cpr::status::HTTP_OK) {
        // Cleanup cache folder
        removeModelCacheFolder();

        // Inform the user about the error and print out model description
        // for easier debugging
        throw std::runtime_error("Failed to download model\n" + modelDescription.toString() + "\nError: " + downloadResponse.text
                                 + "\n HTTP status code: " + std::to_string(downloadResponse.status_code));
    }

    // Save file to cache folder
    std::string folder = getModelCacheFolderPath(cacheDirectory);
    std::string downloadFilename = getFilenameFromUrl(link);
    std::string tarPath = combinePaths(folder, downloadFilename);
    std::ofstream outStream(tarPath, std::ios::binary);
    outStream.write(downloadResponse.text.c_str(), downloadResponse.text.size());
    outStream.close();
}

std::string ZooManager::loadModelFromCache() const {
    const std::string cacheFolder = getModelCacheFolderPath(cacheDirectory);

    // Make sure the cache folder exists
    if(!checkExists(cacheFolder)) throw std::runtime_error("Cache folder " + cacheFolder + " not found.");

    // Find all files in cache folder
    std::vector<std::string> folderFiles = getFilesInFolder(cacheFolder);

    // Make sure there are files in the folder
    if(folderFiles.size() == 0) throw std::runtime_error("No files found in cache folder " + cacheFolder);

    // Return absolute path to the first file found
    return std::filesystem::absolute(folderFiles[0]).string();
}

std::string ZooManager::getFilenameFromUrl(const std::string& url) const {
    // Example url: https://storage.googleapis.com/luxonis/bla/resnet18_rvc2.blob?a=33a&b=443b&last=elon
    // It's fake ^^^ :) but it's just to show how the function works

    // Find query string start
    size_t queryStart = url.find('?');
    bool queryFound = queryStart != std::string::npos;
    if(!queryFound) queryStart = url.size();

    // Cut off query string
    const std::string urlWithoutQuery = url.substr(0, queryStart);

    // Find last slash
    size_t lastSlash = urlWithoutQuery.find_last_of('/');

    // Extract filename
    const std::string filename = urlWithoutQuery.substr(lastSlash + 1);
    return filename;
}

std::vector<std::string> ZooManager::getFilesInFolder(const std::string& folder) const {
    std::vector<std::string> files;
    for(const auto& entry : std::filesystem::directory_iterator(folder)) {
        files.push_back(entry.path().string());
    }
    return files;
}

std::string getModelFromZoo(const NNModelDescription& modelDescription, const std::string& cacheDirectory, bool useCached, bool verbose) {
    // Initialize ZooManager
    ZooManager zooManager(modelDescription, cacheDirectory);

    // Check if model is cached
    bool modelIsCached = zooManager.isCached();
    bool useCachedModel = useCached && modelIsCached;

    // Use cached model if present and useCached is true
    if(useCachedModel) {
        auto modelPath = zooManager.loadModelFromCache();
        if(verbose) std::cout << "Using cached model located at " << modelPath << std::endl;
        return modelPath;
    }

    // Remove cached model if present
    if(modelIsCached) {
        zooManager.removeModelCacheFolder();
    }

    // Create cache folder
    zooManager.createCacheFolder();

    // Download model
    if(verbose) std::cout << "Downloading model from model zoo" << std::endl;
    zooManager.downloadModel();

    // Find path to model in cache
    std::string modelPath = zooManager.loadModelFromCache();
    return modelPath;
}

void downloadModelsFromZoo(const std::string& path, const std::string& cacheDirectory, bool verbose) {
    // Make sure 'path' exists
    if(!std::filesystem::exists(path)) throw std::runtime_error("Path does not exist: " + path);

    // Find all yaml files in 'path'
    std::vector<std::string> yamlFiles;
    for(const auto& entry : std::filesystem::directory_iterator(path)) {
        std::string filePath = entry.path().string();
        if(utility::isYamlFile(filePath)) {
            yamlFiles.push_back(filePath);
        }
    }

    // Download models from yaml files
    for(size_t i = 0; i < yamlFiles.size(); ++i) {
        // Parse yaml file
        const std::string& yamlFile = yamlFiles[i];
        auto modelDescription = NNModelDescription::fromYaml(yamlFile);

        // Download model - ignore the returned model path here == we are only interested in downloading the model
        bool errorOccurred = false;
        try {
            getModelFromZoo(modelDescription, cacheDirectory, false);
        } catch(const std::exception& e) {
            std::cerr << "Failed to download model [" << i + 1 << "/" << yamlFiles.size() << "]:\n" << e.what() << std::endl;
            errorOccurred = true;
        }

        // Print verbose output
        if(verbose && !errorOccurred) {
            std::cout << "Downloaded model [" << i + 1 << "/" << yamlFile.size() << "]:\n" << modelDescription << std::endl;
        }
    }
}

}  // namespace dai