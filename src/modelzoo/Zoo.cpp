#include "depthai/modelzoo/Zoo.hpp"

#include <cpr/cpr.h>

#include <iostream>
#include <nlohmann/json.hpp>

#include "../utility/YamlHelpers.hpp"
#include "../utility/sha1.hpp"

namespace dai {

std::string ZooManager::computeModelHash() const {
    SHA1 hasher;
    hasher.update(modelDescription.toString());
    return hasher.final();
}

std::string ZooManager::getModelCacheFolderName() const {
    return computeModelHash();
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

bool ZooManager::isModelCached() const {
    return checkExists(getModelCacheFolderPath(cacheDirectory));
}

bool ZooManager::checkExists(const std::string& path) const {
    return std::filesystem::exists(path);
}

void ZooManager::downloadModel() {
    // graphql query to send to Hub - always the same
    constexpr std::string_view MODEL_ZOO_QUERY = "query MlDownloads($input: MlModelDownloadsInput!) {ml { modelDownloads(input : $input) { data }}}";

    // Setup request body
    nlohmann::json requestBody;
    requestBody["query"] = MODEL_ZOO_QUERY;

    // Add REQUIRED parameters
    requestBody["variables"]["input"]["platform"] = modelDescription.platform;
    requestBody["variables"]["input"]["modelSlug"] = modelDescription.modelSlug;

    // Add OPTIONAL parameters
    if(!modelDescription.modelVersionSlug.empty()) requestBody["variables"]["input"]["modelVersionSlug"] = modelDescription.modelVersionSlug;

    // Send HTTP request to Hub
    cpr::Response response = cpr::Post(cpr::Url{MODEL_ZOO_URL}, cpr::Body{requestBody.dump()});
    if(checkIsErrorHub(response)) {
        removeModelCacheFolder();
        throw std::runtime_error(generateErrorMessageHub(response));
    }

    // Extract download link from response
    nlohmann::json responseJson = nlohmann::json::parse(response.text);
    auto downloadLinks = responseJson["data"]["ml"]["modelDownloads"]["data"].get<std::vector<std::string>>();

    // std::vector<std::string> downloadLinks = responseJson["data"]["ml"]["modelDownloads"];

    // Download all files and store them in cache folder
    for(const auto& downloadLink : downloadLinks) {
        cpr::Response downloadResponse = cpr::Get(downloadLink);
        if(checkIsErrorModelDownload(downloadResponse)) {
            removeModelCacheFolder();
            throw std::runtime_error(generateErrorMessageModelDownload(downloadResponse));
        }

        // Save downloaded file to cache folder
        std::string filename = getFilenameFromUrl(downloadLink);
        std::string filepath = combinePaths(getModelCacheFolderPath(cacheDirectory), filename);
        std::ofstream file(filepath, std::ios::binary);
        file.write(downloadResponse.text.c_str(), downloadResponse.text.size());
        file.close();
    }
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
    // For this example url, the function would return "resnet18_rvc2.blob"

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

std::string getModelFromZoo(const NNModelDescription& modelDescription, bool useCached, const std::string& cacheDirectory, bool verbose) {
    // Initialize ZooManager
    ZooManager zooManager(modelDescription, cacheDirectory);

    // Check if model is cached
    bool modelIsCached = zooManager.isModelCached();
    bool useCachedModel = useCached && modelIsCached;

    // Use cached model if present and useCached is true
    if(useCachedModel) {
        std::string modelPath = zooManager.loadModelFromCache();
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

bool ZooManager::checkIsErrorHub(const cpr::Response& response) const {
    // Check if response is an HTTP error
    if(response.status_code != cpr::status::HTTP_OK) return true;

    // If there was no HTTP error, check response content for errors
    nlohmann::json responseJson = nlohmann::json::parse(response.text);
    if(responseJson.contains("errors")) return true;
    if(responseJson["data"]["ml"]["modelDownloads"].is_null()) return true;

    // All checks passed - no errors yay
    return false;
}

std::string ZooManager::generateErrorMessageHub(const cpr::Response& response) const {
    std::string errorMessage = "";
    errorMessage += "There was an error while sending a request to the Hub\n";
    errorMessage += "HTTP status code: " + std::to_string(response.status_code) + "\n";

    if(response.status_code == cpr::status::HTTP_OK) {
        nlohmann::json responseJson = nlohmann::json::parse(response.text);
        if(responseJson.contains("errors")) {
            errorMessage += "Errors: " + responseJson["errors"].dump() + "\n";
        }

        if(!responseJson["data"].is_null()) {
            errorMessage += "No model found for the given model description\n";
        }
    }

    return errorMessage;
}

bool ZooManager::checkIsErrorModelDownload(const cpr::Response& response) const {
    bool isError = response.status_code != cpr::status::HTTP_OK;
    return isError;
}

std::string ZooManager::generateErrorMessageModelDownload(const cpr::Response& response) const {
    std::string errorMessage = "";
    errorMessage += "There was an error while downloading the model\n";
    errorMessage += "HTTP status code: " + std::to_string(response.status_code) + "\n";
    return errorMessage;
}

void downloadModelsFromZoo(const std::string& path, const std::string& cacheDirectory, bool verbose) {
    if(verbose) std::cout << "Downloading models from zoo" << std::endl;

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

        // Download model - ignore the returned model path here == we are only interested in downloading the model
        try {
            auto modelDescription = NNModelDescription::fromYamlFile(yamlFile);
            getModelFromZoo(modelDescription, false, cacheDirectory);
            if(verbose) std::cout << "Downloaded model [" << i + 1 << "/" << yamlFiles.size() << "]: " << yamlFile << std::endl;
        } catch(const std::exception& e) {
            std::cerr << "Failed to download model [" << i + 1 << "/" << yamlFiles.size() << "]: " << yamlFile << "\n" << e.what() << std::endl;
        }
    }
}

}  // namespace dai