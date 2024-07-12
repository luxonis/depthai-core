#include "depthai/modelzoo/Zoo.hpp"

#include <filesystem>
#include <iostream>
#include <nlohmann/json.hpp>

#include "../utility/YamlHelpers.hpp"
#include "../utility/sha1.hpp"
#include "utility/Logging.hpp"

#ifdef DEPTHAI_ENABLE_CURL
    #include <cpr/cpr.h>
namespace dai {
class ZooManager {
   public:
    /**
     * @brief Construct a new Zoo Manager object
     *
     * @param modelDescription: Model description
     * @param cacheDirectory: Cache directory, default is ".depthai_cached_models"
     */
    explicit ZooManager(NNModelDescription modelDescription, std::string cacheDirectory = MODEL_ZOO_DEFAULT_CACHE_DIRECTORY)
        : modelDescription(std::move(modelDescription)), cacheDirectory(std::move(cacheDirectory)) {}

    /**
     * @brief Compute hash based on name, version and platform
     *
     * @return std::string: Hash value
     */
    std::string computeModelHash() const;

    /**
     * @brief Get name of the folder where model is cached
     *
     * @return std::string: Cache folder name
     */
    std::string getModelCacheFolderName() const;

    /**
     * @brief Get path to the folder where model is cached
     *
     * @param cacheDirectory: Cache directory where the cached models are stores
     * @return std::string: Cache folder name
     */
    std::string getModelCacheFolderPath(const std::string& cacheDirectory) const;

    /**
     * @brief Create cache folder
     */
    void createCacheFolder() const;

    /**
     * @brief Remove cache folder where the model is cached
     */
    void removeModelCacheFolder() const;

    /**
     * @brief Check if model is cached
     *
     * @return bool: True if model is cached
     */
    bool isModelCached() const;

    /**
     * @brief Download model from model zoo
     */
    void downloadModel();

    /**
     * @brief Return path to model in cache
     *
     * @return std::string: Path to model
     */
    std::string loadModelFromCache() const;

   private:
    // Description of the model
    NNModelDescription modelDescription;

    // Path to directory where to store the cached models
    std::string cacheDirectory;
};

std::string generateErrorMessageHub(const cpr::Response& response) {
    std::string errorMessage;
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

bool checkIsErrorModelDownload(const cpr::Response& response) {
    bool isError = response.status_code != cpr::status::HTTP_OK;
    return isError;
}

std::string generateErrorMessageModelDownload(const cpr::Response& response) {
    std::string errorMessage;
    errorMessage += "There was an error while downloading the model\n";
    errorMessage += "HTTP status code: " + std::to_string(response.status_code) + "\n";
    return errorMessage;
}

std::string combinePaths(const std::string& path1, const std::string& path2) {
    return std::filesystem::path(path1).append(path2).string();
}

bool checkIsErrorHub(const cpr::Response& response) {
    // Check if response is an HTTP error
    if(response.status_code != cpr::status::HTTP_OK) return true;

    // If there was no HTTP error, check response content for errors
    nlohmann::json responseJson = nlohmann::json::parse(response.text);
    if(responseJson.contains("errors")) return true;
    if(responseJson["data"]["ml"]["modelDownloads"].is_null()) return true;

    // All checks passed - no errors yay
    return false;
}

std::vector<std::string> getFilesInFolder(const std::string& folder) {
    std::vector<std::string> files;
    for(const auto& entry : std::filesystem::directory_iterator(folder)) {
        files.push_back(entry.path().string());
    }
    return files;
}

std::string getFilenameFromUrl(const std::string& url) {
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

void ZooManager::createCacheFolder() const {
    std::string cacheFolderName = getModelCacheFolderPath(cacheDirectory);
    std::filesystem::create_directories(cacheFolderName);
}

void ZooManager::removeModelCacheFolder() const {
    std::string cacheFolderName = getModelCacheFolderPath(cacheDirectory);
    std::filesystem::remove_all(cacheFolderName);
}

bool ZooManager::isModelCached() const {
    return std::filesystem::exists(getModelCacheFolderPath(cacheDirectory));
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
    if(!modelDescription.modelInstanceSlug.empty()) requestBody["variables"]["input"]["modelInstanceSlug"] = modelDescription.modelInstanceSlug;
    if(!modelDescription.modelInstanceHash.empty()) requestBody["variables"]["input"]["modelInstanceHash"] = modelDescription.modelInstanceHash;
    if(!modelDescription.optimizationLevel.empty()) requestBody["variables"]["input"]["optimizationLevel"] = modelDescription.optimizationLevel;
    if(!modelDescription.compressionLevel.empty()) requestBody["variables"]["input"]["compressionLevel"] = modelDescription.compressionLevel;

    // Send HTTP request to Hub
    cpr::Response response = cpr::Post(cpr::Url{MODEL_ZOO_URL}, cpr::Body{requestBody.dump()});
    if(checkIsErrorHub(response)) {
        removeModelCacheFolder();
        throw std::runtime_error(generateErrorMessageHub(response));
    }

    // Extract download link from response
    nlohmann::json responseJson = nlohmann::json::parse(response.text);
    auto downloadLinks = responseJson["data"]["ml"]["modelDownloads"]["data"].get<std::vector<std::string>>();

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
    if(!std::filesystem::exists(cacheFolder)) throw std::runtime_error("Cache folder " + cacheFolder + " not found.");

    // Find all files in cache folder
    std::vector<std::string> folderFiles = getFilesInFolder(cacheFolder);

    // Make sure there are files in the folder
    if(folderFiles.empty()) throw std::runtime_error("No files found in cache folder " + cacheFolder);

    // Return absolute path to the first file found
    return std::filesystem::absolute(folderFiles[0]).string();
}

std::string getModelFromZoo(const NNModelDescription& modelDescription, bool useCached, const std::string& cacheDirectory) {
    // Initialize ZooManager
    ZooManager zooManager(modelDescription, cacheDirectory);

    // Check if model is cached
    bool modelIsCached = zooManager.isModelCached();
    bool useCachedModel = useCached && modelIsCached;

    // Use cached model if present and useCached is true
    if(useCachedModel) {
        std::string modelPath = zooManager.loadModelFromCache();
        Logging::getInstance().logger.info("Using cached model located at {}", modelPath);
        return modelPath;
    }

    // Remove cached model if present
    if(modelIsCached) {
        zooManager.removeModelCacheFolder();
    }

    // Create cache folder
    zooManager.createCacheFolder();

    // Download model
    Logging::getInstance().logger.info("Downloading model from model zoo");
    zooManager.downloadModel();

    // Find path to model in cache
    std::string modelPath = zooManager.loadModelFromCache();
    return modelPath;
}

void downloadModelsFromZoo(const std::string& path, const std::string& cacheDirectory) {
    Logging::getInstance().logger.info("Downloading models from zoo");
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
            getModelFromZoo(modelDescription, true, cacheDirectory);
            Logging::getInstance().logger.info("Downloaded model [{} / {}]: {}", i + 1, yamlFiles.size(), yamlFile);
        } catch(const std::exception& e) {
            Logging::getInstance().logger.error("Failed to download model [{} / {}]: {}\n{}", i + 1, yamlFiles.size(), yamlFile, e.what());
        }
    }
}
}  // namespace dai

#else
namespace dai {
std::string getModelFromZoo(const NNModelDescription& modelDescription, bool useCached, const std::string& cacheDirectory) {
    (void)modelDescription;
    (void)useCached;
    (void)cacheDirectory;
    throw std::runtime_error("getModelFromZoo requires libcurl to be enabled. Please recompile DepthAI with libcurl enabled.");
}

void downloadModelsFromZoo(const std::string& path, const std::string& cacheDirectory) {
    (void)path;
    (void)cacheDirectory;
    throw std::runtime_error("downloadModelsFromZoo requires libcurl to be enabled. Please recompile DepthAI with libcurl enabled.");
}
}  // namespace dai
#endif
