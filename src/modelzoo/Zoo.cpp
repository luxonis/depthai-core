#include "depthai/modelzoo/Zoo.hpp"

#include <filesystem>
#include <iostream>
#include <nlohmann/json.hpp>

#include "../utility/YamlHelpers.hpp"
#include "../utility/sha1.hpp"
#include "../utility/Environment.hpp"
#include "utility/Logging.hpp"

#ifdef DEPTHAI_ENABLE_CURL
    #include <cpr/api.h>
    #include <cpr/parameters.h>
    #include <cpr/status_codes.h>
namespace dai {
class ZooManager {
   public:
    /**
     * @brief Construct a new Zoo Manager object
     *
     * @param modelDescription: Model description
     * @param cacheDirectory: Cache directory, default is ".depthai_cached_models"
     */
    explicit ZooManager(NNModelDescription modelDescription, std::string cacheDirectory = MODEL_ZOO_DEFAULT_CACHE_DIRECTORY, std::string apiKey = "")
        : modelDescription(std::move(modelDescription)), apiKey(std::move(apiKey)), cacheDirectory(std::move(cacheDirectory)) {
        // If the API is empty override from environment variable, if it exists
        if(this->apiKey.empty()) {
            logger::info("Trying to get API key from environment variable DEPTHAI_HUB_API_KEY");
            auto envApiKey = utility::getEnvAs<std::string>("DEPTHAI_HUB_API_KEY", "");
            if(!envApiKey.empty()) {
                this->apiKey = envApiKey;
                logger::info("API key found in environment variable DEPTHAI_HUB_API_KEY");
            } else {
                logger::info("API key not provided");
            }
        } else {
            logger::info("API key provided");
        }
    }

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
    void downloadModel(const nlohmann::json& responseJson);

    /**
     * @brief Return path to model in cache
     *
     * @return std::string: Path to model
     */
    std::string loadModelFromCache() const;

    /**
     * @brief Get path to metadata file
     *
     * @return std::string: Path to metadata file
     */
    std::string getMetadataFilePath() const;

    /**
     * @brief Fetch model download links from Hub
     *
     * @return nlohmann::json: JSON with download links
     */
    nlohmann::json fetchModelDownloadLinks();

    /**
     * @brief Get files in folder
     *
     * @return std::vector<std::string>: Files in folder
     */
    std::vector<std::string> getFilesInFolder(const std::string& folder) const;

    /**
     * @brief Check if internet is available
     *
     * @return bool: True if internet is available
     */
    static bool connectionToZooAvailable(); 

   private:
    // Description of the model
    NNModelDescription modelDescription;

    // Private key to access the Hub
    std::string apiKey;

    // Path to directory where to store the cached models
    std::string cacheDirectory;
};

std::string generateErrorMessageHub(const cpr::Response& response) {
    std::string errorMessage;
    errorMessage += "There was an error while sending a request to the Hub\n";
    errorMessage += "HTTP status code: " + std::to_string(response.status_code) + "\n";
    errorMessage += "CPR error code: " + std::to_string(static_cast<int>(response.error.code)) + "\n";

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
    errorMessage += "CPR error code: " + std::to_string(static_cast<int>(response.error.code)) + "\n";
    return errorMessage;
}

std::string combinePaths(const std::string& path1, const std::string& path2) {
    return std::filesystem::path(path1).append(path2).string();
}

bool checkIsErrorHub(const cpr::Response& response) {
    // Check if response is an HTTP error
    if(response.status_code != cpr::status::HTTP_OK) return true;

    // If there was no HTTP error, check presence of required fields
    nlohmann::json responseJson = nlohmann::json::parse(response.text);
    if(!responseJson.contains("hash")) return true;
    if(!responseJson.contains("download_links")) return true;

    // All checks passed - no errors yay
    return false;
}

std::vector<std::string> ZooManager::getFilesInFolder(const std::string& folder) const {
    auto metadata = utility::loadYaml(getMetadataFilePath());
    auto downloadedFiles = utility::yamlGet<std::vector<std::string>>(metadata, "downloaded_files");
    std::vector<std::string> files;
    for(const auto& downloadedFile : downloadedFiles) {
        if(std::filesystem::exists(combinePaths(folder, downloadedFile))) {
            files.push_back(combinePaths(folder, downloadedFile));
        }
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

nlohmann::json ZooManager::fetchModelDownloadLinks() {
    // Add request parameters
    cpr::Parameters params;

    // Required parameters
    // clang-format off
    std::vector<std::pair<std::string, std::string>> requiredParams = {
        {"slug", modelDescription.model},
        {"platform", modelDescription.platform}
    };
    // clang-format on
    for(const auto& param : requiredParams) {
        params.Add({param.first, param.second});
    }

    // Optional parameters
    // clang-format off
    std::vector<std::pair<std::string, std::string>> optionalParams = {
        {"optimizationLevel", modelDescription.optimizationLevel},
        {"compressionLevel", modelDescription.compressionLevel},
        {"snpeVersion", modelDescription.snpeVersion},
        {"modelPrecisionType", modelDescription.modelPrecisionType}
    };
    // clang-format on
    for(const auto& param : optionalParams) {
        if(!param.second.empty()) {
            params.Add({param.first, param.second});
        }
    }

    // Set the Authorization headers
    cpr::Header headers = {
        {"Content-Type", "application/json"},
    };
    if(!apiKey.empty()) {
        headers["Authorization"] = "Bearer " + apiKey;
    }

    // Send HTTP GET request to REST endpoint
    cpr::Response response = cpr::Get(cpr::Url{MODEL_ZOO_DOWNLOAD_ENDPOINT}, headers, params);
    if(checkIsErrorHub(response)) {
        removeModelCacheFolder();
        throw std::runtime_error(generateErrorMessageHub(response));
    }

    // Extract download links from response
    nlohmann::json responseJson = nlohmann::json::parse(response.text);
    return responseJson;
}

void ZooManager::downloadModel(const nlohmann::json& responseJson) {
    // Extract download links from response
    auto downloadLinks = responseJson["download_links"].get<std::vector<std::string>>();
    auto downloadHash = responseJson["hash"].get<std::string>();

    // Metadata
    YAML::Node metadata;
    metadata["hash"] = downloadHash;
    metadata["downloaded_files"] = std::vector<std::string>();

    // Download all files and store them in cache folder
    for(const auto& downloadLink : downloadLinks) {
        cpr::Response downloadResponse = cpr::Get(cpr::Url(downloadLink));
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

        // Add filename to metadata
        metadata["downloaded_files"].push_back(filename);
    }

    // Save metadata to file
    utility::saveYaml(metadata, getMetadataFilePath());
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

std::string getModelFromZoo(const NNModelDescription& modelDescription, bool useCached, const std::string& cacheDirectory, const std::string& apiKey) {
    // Initialize ZooManager
    ZooManager zooManager(modelDescription, cacheDirectory, apiKey);

    // Check if model is cached
    bool modelIsCached = zooManager.isModelCached();
    bool isMetadataPresent = std::filesystem::exists(zooManager.getMetadataFilePath());
    bool useCachedModel = useCached && modelIsCached && isMetadataPresent;

    bool performInternetCheck = utility::getEnvAs<bool>("DEPTHAI_ZOO_INTERNET_CHECK", true); // default is true

    // Check if internet is available
    bool internetIsAvailable = performInternetCheck && ZooManager::connectionToZooAvailable();
    nlohmann::json responseJson;

    if(internetIsAvailable) {
        responseJson = zooManager.fetchModelDownloadLinks();
    }

    // Use cached model if present and useCached is true
    if(useCachedModel) {
        if(!internetIsAvailable) {
            std::string modelPath = zooManager.loadModelFromCache();
            logger::info("Using cached model located at {}", modelPath);
            return modelPath;
        }

        auto responseHash = responseJson["hash"].get<std::string>();
        auto metadata = utility::loadYaml(zooManager.getMetadataFilePath());
        auto metadataHash = utility::yamlGet<std::string>(metadata, "hash");

        if(responseHash == metadataHash) {
            std::string modelPath = zooManager.loadModelFromCache();
            logger::info("Using cached model located at {}", modelPath);
            return modelPath;
        }

        logger::warn("Cached model hash does not match response hash, downloading anew ...");
    }

    // Remove cached model if present
    if(modelIsCached) {
        zooManager.removeModelCacheFolder();
    }

    // Model is not cached and internet check is disabled
    if(!performInternetCheck) {
        throw std::runtime_error("Model no available. Please set DEPTHAI_ZOO_INTERNET_CHECK to 1 to download models from the model zoo.");
    }

    // Model is not cached and internet check is enabled but no internet connection is available
    if(performInternetCheck && !internetIsAvailable) {
        throw std::runtime_error("Model not available. No internet connection available. Could not connect to host: " + std::string(MODEL_ZOO_HEALTH_ENDPOINT));
    }

    // Create cache folder
    zooManager.createCacheFolder();

    // Download model
    logger::info("Downloading model from model zoo");
    zooManager.downloadModel(responseJson);

    // Find path to model in cache
    std::string modelPath = zooManager.loadModelFromCache();
    return modelPath;
}

void downloadModelsFromZoo(const std::string& path, const std::string& cacheDirectory, const std::string& apiKey) {
    logger::info("Downloading models from zoo");
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
            getModelFromZoo(modelDescription, true, cacheDirectory, apiKey);
            logger::info("Downloaded model [{} / {}]: {}", i + 1, yamlFiles.size(), yamlFile);
        } catch(const std::exception& e) {
            logger::error("Failed to download model [{} / {}]: {}\n{}", i + 1, yamlFiles.size(), yamlFile, e.what());
        }
    }
}

bool ZooManager::connectionToZooAvailable() {
    
    int timeoutMs = utility::getEnvAs<int>("DEPTHAI_ZOO_INTERNET_CHECK_TIMEOUT", 1000); // default is 1000ms
    constexpr std::string_view host = MODEL_ZOO_HEALTH_ENDPOINT;

    // Check if internet is available
    bool connected = false;
    try {
        cpr::Response r = cpr::Get(cpr::Url{host}, cpr::Timeout{timeoutMs});
        connected = r.status_code == cpr::status::HTTP_OK;
    } catch(const cpr::Error& e) {
        // pass, we don't care about the error
    }

    return connected;
}

std::string ZooManager::getMetadataFilePath() const {
    return combinePaths(getModelCacheFolderPath(cacheDirectory), "metadata.yaml");
}

}  // namespace dai

#else
namespace dai {
std::string getModelFromZoo(const NNModelDescription& modelDescription, bool useCached, const std::string& cacheDirectory, const std::string& apiKey) {
    (void)modelDescription;
    (void)useCached;
    (void)cacheDirectory;
    (void)apiKey;
    throw std::runtime_error("getModelFromZoo requires libcurl to be enabled. Please recompile DepthAI with libcurl enabled.");
}

void downloadModelsFromZoo(const std::string& path, const std::string& cacheDirectory, const std::string& apiKey) {
    (void)path;
    (void)cacheDirectory;
    (void)apiKey;
    throw std::runtime_error("downloadModelsFromZoo requires libcurl to be enabled. Please recompile DepthAI with libcurl enabled.");
}
}  // namespace dai
#endif
