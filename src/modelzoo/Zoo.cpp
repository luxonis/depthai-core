#include "depthai/modelzoo/Zoo.hpp"

#if defined(WIN32) || defined(_WIN32)
    #include <cwctype>
#else
    #include <cctype>
#endif

#include <fmt/format.h>
#include <fmt/std.h>

#include <filesystem>
#include <iostream>
#include <memory>
#include <mutex>
#include <nlohmann/json.hpp>
#include <nlohmann/json_fwd.hpp>
#include <optional>
#include <string_view>

#include "utility/Environment.hpp"
#include "utility/Logging.hpp"
#include "utility/Platform.hpp"
#include "utility/Telemetry.hpp"
#include "utility/YamlHelpers.hpp"
#include "utility/sha1.hpp"

#ifdef DEPTHAI_ENABLE_CURL
    #include <cpr/api.h>
    #include <cpr/cprtypes.h>
    #include <cpr/parameters.h>
    #include <cpr/status_codes.h>
#endif

namespace dai {

namespace fs = std::filesystem;

static std::mutex modelZooSettingsMutex;  // Guards the settings copies only; never hold it across model I/O
static std::string modelZooHealthEndpoint = "https://easyml.cloud.luxonis.com/models/api/v1/health/";
static std::string modelZooDownloadEndpoint = "https://easyml.cloud.luxonis.com/models/api/v1/models/download";
static fs::path modelZooDefaultCachePath = dai::platform::getDaiCacheDir() / "models";
static fs::path modelZooDefaultModelsPath = "depthai_models";  // folder

#ifdef DEPTHAI_ENABLE_CURL
class ZooManager {
   public:
    explicit ZooManager(NNModelDescription modelDescription, fs::path cacheDirectory = "", const std::string& apiKey = "")
        : modelDescription(std::move(modelDescription)), apiKey(std::move(apiKey)), cacheDirectory(std::move(cacheDirectory)) {
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

        if(this->cacheDirectory.empty()) {
            logger::info("Trying to get cache directory from environment variable DEPTHAI_ZOO_CACHE_PATH");
            this->cacheDirectory = utility::getEnvAs<fs::path>("DEPTHAI_ZOO_CACHE_PATH", dai::modelzoo::getDefaultCachePath(), false);
        }

        if(this->cacheDirectory.empty()) {
            throw std::runtime_error("Cache directory is not set");
        }

        bool cacheDirectoryExists = platform::checkPathExists(this->cacheDirectory);
        if(!cacheDirectoryExists) {
            try {
                logger::debug("Cache directory does not exist, creating it: {}", this->cacheDirectory.string());
                std::filesystem::create_directories(this->cacheDirectory);
            } catch(const std::exception& e) {
                throw std::runtime_error(fmt::format("Failed to create cache directory: {} | {}", this->cacheDirectory.string(), e.what()));
            }
        }

        bool hasReadPermissions = platform::checkReadPermissions(this->cacheDirectory);
        bool hasWritePermissions = platform::checkWritePermissions(this->cacheDirectory);
        logger::debug("Cache directory has read permissions: {}, has write permissions: {}", hasReadPermissions, hasWritePermissions);

        if(!hasReadPermissions) {
            throw std::runtime_error(fmt::format("Cache directory {} is not readable", this->cacheDirectory.string()));
        }

        if(hasWritePermissions) {
            logger::info("Cache directory has write permissions, creating a .locks folder");
            createFolder(".locks");
            const fs::path modelLockFilePath = platform::joinPaths(platform::joinPaths(this->cacheDirectory, ".locks"), getModelCacheFolderName() + ".lock");
            logger::info("Locking model cache directory: {}", modelLockFilePath);
            cacheFolderLock = platform::FileLock::lock(modelLockFilePath, true);
            logger::info("Model cache directory locked: {}", modelLockFilePath);
        } else {
            logger::info("Cache directory does not have write permissions, skipping lock creation");
        }
    }

    std::string computeModelHash() const;

    std::string getModelCacheFolderName() const;

    fs::path getModelCacheFolderPath(const fs::path& cacheDirectory) const;

    void createFolder(const std::string& folderName) const;

    void removeModelCacheFolder() const;

    bool isModelCached() const;

    void downloadModel(const nlohmann::json& responseJson, std::unique_ptr<cpr::ProgressCallback> cprCallback);

    fs::path loadModelFromCache() const;

    fs::path getMetadataFilePath() const;

    fs::path getGlobalMetadataFilePath() const;

    nlohmann::json fetchModelDownloadLinks();

    nlohmann::json fetchModelDetails(const std::string& modelId) const;

    std::optional<nlohmann::json> getModelLoadedTelemetryProperties(const nlohmann::json& responseJson) const;

    std::vector<fs::path> getFilesInFolder(const fs::path& folder) const;

    static bool connectionToZooAvailable();

    NNModelDescription modelDescription;

    std::string apiKey;

    fs::path cacheDirectory;

    std::unique_ptr<platform::FileLock> cacheFolderLock;
};

#endif

fs::path getYamlFilePath(const std::string& name, const fs::path& modelsPath = "");

#ifdef DEPTHAI_ENABLE_CURL
std::string generateErrorMessageHub(const cpr::Response& response) {
    std::string errorMessage;
    errorMessage += "There was an error while sending a request to the Hub\n";
    errorMessage += "HTTP status code: " + std::to_string(response.status_code) + "\n";
    errorMessage += "CPR error code: " + std::to_string(static_cast<int>(response.error.code)) + "\n";
    errorMessage += "Response text: " + response.text + "\n";
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
    errorMessage += "Response text: " + response.text + "\n";
    return errorMessage;
}

bool checkIsErrorHub(const cpr::Response& response) {
    if(response.status_code != cpr::status::HTTP_OK) return true;

    nlohmann::json responseJson = nlohmann::json::parse(response.text);
    if(!responseJson.contains("hash")) return true;
    if(!responseJson.contains("download_links")) return true;

    return false;
}

cpr::Header makeHubHeaders(const std::string& apiKey) {
    cpr::Header headers = {
        {"Content-Type", "application/json"},
    };
    if(!apiKey.empty()) {
        headers["Authorization"] = "Bearer " + apiKey;
    }
    return headers;
}

std::string getModelDetailsEndpoint(const std::string& modelId) {
    auto endpoint = dai::modelzoo::getDownloadEndpoint();
    while(!endpoint.empty() && endpoint.back() == '/') {
        endpoint.pop_back();
    }

    constexpr std::string_view downloadSuffix = "/download";
    if(endpoint.size() >= downloadSuffix.size() && endpoint.compare(endpoint.size() - downloadSuffix.size(), downloadSuffix.size(), downloadSuffix) == 0) {
        endpoint.erase(endpoint.size() - downloadSuffix.size());
    }

    return endpoint + "/" + modelId;
}

void emitZooModelLoadedTelemetry(std::optional<nlohmann::json> properties, bool fromCache) {
    if(!properties.has_value()) {
        return;
    }

    try {
        (*properties)["from_cache"] = fromCache;
        utility::Telemetry::getInstance().event("depthai_zoo_model_loaded", std::move(*properties));
    } catch(const std::exception& ex) {
        logger::debug("Failed to emit zoo model loaded telemetry: {}", ex.what());
    }
}

std::vector<fs::path> ZooManager::getFilesInFolder(const fs::path& folder) const {
    auto metadata = utility::loadYaml(getMetadataFilePath());
    auto downloadedFiles = utility::yamlGet<std::vector<std::string>>(metadata, "downloaded_files");
    std::vector<fs::path> files;
    for(const auto& downloadedFile : downloadedFiles) {
        if(std::filesystem::exists(folder / downloadedFile)) {
            files.push_back(folder / downloadedFile);
        }
    }
    return files;
}

std::string getFilenameFromUrl(const std::string& url) {
    size_t queryStart = url.find('?');
    bool queryFound = queryStart != std::string::npos;
    if(!queryFound) queryStart = url.size();

    const std::string urlWithoutQuery = url.substr(0, queryStart);

    size_t lastSlash = urlWithoutQuery.find_last_of('/');

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

fs::path ZooManager::getModelCacheFolderPath(const fs::path& cacheDirectory) const {
    return cacheDirectory / getModelCacheFolderName();
}

void ZooManager::removeModelCacheFolder() const {
    fs::path cacheFolderPath = getModelCacheFolderPath(cacheDirectory);
    std::filesystem::remove_all(cacheFolderPath);

    if(!modelDescription.globalMetadataEntryName.empty()) {
        fs::path globalMetadataPath = getGlobalMetadataFilePath();
        if(std::filesystem::exists(globalMetadataPath)) {
            auto globalMetadata = utility::loadYaml(globalMetadataPath);
            globalMetadata.remove(modelDescription.globalMetadataEntryName);
            utility::saveYaml(globalMetadata, globalMetadataPath);
        }
    }
}

bool ZooManager::isModelCached() const {
    return std::filesystem::exists(getModelCacheFolderPath(cacheDirectory));
}

nlohmann::json ZooManager::fetchModelDownloadLinks() {
    cpr::Parameters params;

    // clang-format off
    std::vector<std::pair<std::string, std::string>> requiredParams = {
        {"slug", modelDescription.model},
        {"platform", modelDescription.platform}
    };
    // clang-format on
    for(const auto& param : requiredParams) {
        params.Add({param.first, param.second});
    }

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

    auto headers = makeHubHeaders(apiKey);

    cpr::Response response = cpr::Get(cpr::Url{dai::modelzoo::getDownloadEndpoint()}, headers, params);
    if(checkIsErrorHub(response)) {
        removeModelCacheFolder();
        throw std::runtime_error(generateErrorMessageHub(response));
    }

    nlohmann::json responseJson = nlohmann::json::parse(response.text);
    return responseJson;
}

nlohmann::json ZooManager::fetchModelDetails(const std::string& modelId) const {
    const auto timeoutMs = utility::getEnvAs<int>("DEPTHAI_ZOO_INTERNET_CHECK_TIMEOUT", 1000);
    cpr::Response response = cpr::Get(cpr::Url{getModelDetailsEndpoint(modelId)}, makeHubHeaders(apiKey), cpr::Timeout{timeoutMs});
    if(response.status_code != cpr::status::HTTP_OK) {
        throw std::runtime_error(generateErrorMessageHub(response));
    }

    auto responseJson = nlohmann::json::parse(response.text);
    if(!responseJson.is_object()) {
        throw std::runtime_error("Model details response is not a JSON object");
    }
    return responseJson;
}

std::optional<nlohmann::json> ZooManager::getModelLoadedTelemetryProperties(const nlohmann::json& responseJson) const {
    const auto modelId = responseJson.value("model_id", std::string{});
    if(modelId.empty()) {
        logger::debug("Skipping zoo model loaded telemetry: download response does not contain model_id");
        return std::nullopt;
    }

    try {
        const auto modelDetails = fetchModelDetails(modelId);
        const auto isOfficial = modelDetails.value("is_official", false);
        return nlohmann::json{
            {"is_official", isOfficial},
            {"model_slug", isOfficial ? modelDetails.value("slug", std::string{}) : "REDACTED"},
            {"team_slug", isOfficial ? modelDetails.value("team_slug", std::string{}) : "REDACTED"},
        };
    } catch(const std::exception& ex) {
        logger::debug("Failed to fetch zoo model details for telemetry: {}", ex.what());
        return std::nullopt;
    }
}

void ZooManager::downloadModel(const nlohmann::json& responseJson, std::unique_ptr<cpr::ProgressCallback> cprCallback) {
    auto downloadLinks = responseJson["download_links"].get<std::vector<std::string>>();
    auto downloadHash = responseJson["hash"].get<std::string>();
    auto modelId = responseJson.value("model_id", "");
    auto modelVersionId = responseJson.value("model_version_id", "");
    auto modelInstanceId = responseJson.value("model_instance_id", "");

    YAML::Node metadata;
    metadata["hash"] = downloadHash;
    metadata["model_id"] = modelId;
    metadata["model_version_id"] = modelVersionId;
    metadata["model_instance_id"] = modelInstanceId;
    metadata["downloaded_files"] = std::vector<std::string>();

    for(const auto& downloadLink : downloadLinks) {
        cpr::Response downloadResponse = cpr::Get(cpr::Url(downloadLink), *cprCallback);
        if(checkIsErrorModelDownload(downloadResponse)) {
            removeModelCacheFolder();
            throw std::runtime_error(generateErrorMessageModelDownload(downloadResponse));
        }

        std::string filename = getFilenameFromUrl(downloadLink);
        fs::path filepath = getModelCacheFolderPath(cacheDirectory) / filename;
        std::ofstream file(filepath, std::ios::binary);
        file.write(downloadResponse.text.c_str(), downloadResponse.text.size());
        file.close();

        metadata["downloaded_files"].push_back(filename);
    }

    utility::saveYaml(metadata, getMetadataFilePath());

    if(!modelDescription.globalMetadataEntryName.empty()) {
        YAML::Node globalMetadata;
        fs::path globalMetadataPath = getGlobalMetadataFilePath();
        if(std::filesystem::exists(globalMetadataPath)) {
            globalMetadata = utility::loadYaml(globalMetadataPath);
        }
        metadata["folder_name"] = getModelCacheFolderName();
        globalMetadata[modelDescription.globalMetadataEntryName] = metadata;
        utility::saveYaml(globalMetadata, globalMetadataPath);
    }
}

fs::path ZooManager::loadModelFromCache() const {
    const fs::path cacheFolder = getModelCacheFolderPath(cacheDirectory);

    if(!std::filesystem::exists(cacheFolder)) throw std::runtime_error(fmt::format("Cache folder {} not found.", cacheFolder));

    std::vector<fs::path> folderFiles = getFilesInFolder(cacheFolder);

    if(folderFiles.empty()) throw std::runtime_error(fmt::format("No files found in cache folder {}", cacheFolder));

    return std::filesystem::absolute(folderFiles[0]);
}

class CprCallback {
   public:
    virtual ~CprCallback() = default;
    CprCallback(const std::string& modelName) : modelName(modelName) {}

    virtual void cprCallback(
        cpr::cpr_off_t downloadTotal, cpr::cpr_off_t downloadNow, cpr::cpr_off_t uploadTotal, cpr::cpr_off_t uploadNow, intptr_t userdata) = 0;

    virtual std::unique_ptr<cpr::ProgressCallback> getCprProgressCallback() {
        return std::make_unique<cpr::ProgressCallback>(
            [this](cpr::cpr_off_t downloadTotal, cpr::cpr_off_t downloadNow, cpr::cpr_off_t uploadTotal, cpr::cpr_off_t uploadNow, intptr_t userdata) {
                this->cprCallback(downloadTotal, downloadNow, uploadTotal, uploadNow, userdata);
                return true;
            });
    }

   protected:
    std::string modelName;
};

class JsonCprCallback : public CprCallback {
    constexpr static long long PRINT_INTERVAL_MS = 100;

   public:
    JsonCprCallback(const std::string& modelName) : CprCallback(modelName) {
        startTime = std::chrono::steady_clock::time_point::min();
    }

    void print(long downloadTotal, long downloadNow, const std::string& modelName) {
        nlohmann::json json = {
            {"download_total", downloadTotal},
            {"download_now", downloadNow},
            {"model_name", modelName},
            {"log_type", "download"},
        };
        std::cout << json.dump() << std::endl;
    }

    void cprCallback(
        cpr::cpr_off_t downloadTotal, cpr::cpr_off_t downloadNow, cpr::cpr_off_t uploadTotal, cpr::cpr_off_t uploadNow, intptr_t userdata) override {
        (void)uploadTotal;
        (void)uploadNow;
        (void)userdata;

        bool firstCall = startTime == std::chrono::steady_clock::time_point::min();
        if(firstCall || downloadTotal == 0) {
            startTime = std::chrono::steady_clock::now();
        }

        bool shouldPrint = std::chrono::steady_clock::now() - startTime > std::chrono::milliseconds(PRINT_INTERVAL_MS) || this->downloadTotal != downloadTotal;

        if(shouldPrint) {
            print(downloadTotal, downloadNow, modelName);
            startTime = std::chrono::steady_clock::now();
        }

        this->downloadTotal = downloadTotal;
        this->downloadNow = downloadNow;
    }

    ~JsonCprCallback() override {
        if(downloadTotal != 0) {
            print(downloadTotal, downloadNow, modelName);
        }
    }

   private:
    long downloadTotal = 0;
    long downloadNow = 0;
    std::chrono::steady_clock::time_point startTime;
};

class PrettyCprCallback : public CprCallback {
   public:
    PrettyCprCallback(const std::string& modelName) : CprCallback(modelName), finalProgressPrinted(false) {}

    void cprCallback(
        cpr::cpr_off_t downloadTotal, cpr::cpr_off_t downloadNow, cpr::cpr_off_t uploadTotal, cpr::cpr_off_t uploadNow, intptr_t userdata) override {
        (void)uploadTotal;
        (void)uploadNow;
        (void)userdata;

        if(finalProgressPrinted) return;

        if(downloadTotal > 0) {
            float progress = static_cast<float>(downloadNow) / downloadTotal;
            int barWidth = 50;
            int pos = static_cast<int>(barWidth * progress);

            std::cout << "\rDownloading " << modelName << " [";
            for(int i = 0; i < barWidth; ++i) {
                if(i < pos)
                    std::cout << "=";
                else if(i == pos)
                    std::cout << ">";
                else
                    std::cout << " ";
            }
            std::cout << "] " << std::fixed << std::setprecision(3) << progress * 100.0f << "% " << downloadNow / 1024.0f / 1024.0f << "/"
                      << downloadTotal / 1024.0f / 1024.0f << " MB";

            if(downloadNow == downloadTotal) {
                std::cout << std::endl;
                finalProgressPrinted = true;
            } else {
                std::cout << "\r";
                std::cout.flush();
            }
        }
    }

   private:
    bool finalProgressPrinted;
};

class NoneCprCallback : public CprCallback {
   public:
    NoneCprCallback(const std::string& modelName) : CprCallback(modelName) {}

    void cprCallback(cpr::cpr_off_t, cpr::cpr_off_t, cpr::cpr_off_t, cpr::cpr_off_t, intptr_t) override {}
};

std::unique_ptr<CprCallback> getCprCallback(const std::string& format, const std::string& name) {
    if(format == "json") {
        return std::make_unique<JsonCprCallback>(name);
    } else if(format == "pretty") {
        return std::make_unique<PrettyCprCallback>(name);
    } else if(format == "none") {
        return std::make_unique<NoneCprCallback>(name);
    }
    throw std::runtime_error("Invalid format: " + format);
}

fs::path getModelFromZoo(
    const NNModelDescription& modelDescription, bool useCached, const fs::path& cacheDirectory, const std::string& apiKey, const std::string& progressFormat) {
    if(!modelDescription.check()) throw std::runtime_error("Invalid model description:\n" + modelDescription.toString());

    ZooManager zooManager(modelDescription, cacheDirectory, apiKey);

    bool hasLock = (zooManager.cacheFolderLock != nullptr);
    bool modelIsCached = zooManager.isModelCached();

    if(!hasLock && modelIsCached) {
        fs::path modelPath = zooManager.loadModelFromCache();
        logger::info("Model is cached but model lock could not be acquired (likely due to insufficient write permissions). Using cached model located at {}",
                     modelPath);
        return modelPath;
    }

    if(!hasLock && !modelIsCached) {
        throw std::runtime_error("Model is not cached and no lock is held. Please check the cache directory permissions.");
    }

    bool isMetadataPresent = std::filesystem::exists(zooManager.getMetadataFilePath());
    bool useCachedModel = useCached && modelIsCached && isMetadataPresent;

    bool performInternetCheck = utility::getEnvAs<bool>("DEPTHAI_ZOO_INTERNET_CHECK", true);  // default is true

    bool internetIsAvailable = performInternetCheck && ZooManager::connectionToZooAvailable();
    nlohmann::json responseJson;
    std::optional<nlohmann::json> modelLoadedTelemetryProperties;

    logger::info(
        "Model is cached: {} | Metadata present: {} | Use cached model: {} | Perform internet check: {} | Internet is available: {} | useCached: {} | has "
        "folder lock: {}",
        modelIsCached,
        isMetadataPresent,
        useCachedModel,
        performInternetCheck,
        internetIsAvailable,
        useCached,
        hasLock);

    if(internetIsAvailable) {
        responseJson = zooManager.fetchModelDownloadLinks();
        if(utility::Telemetry::isTelemetryEnabled()) {
            modelLoadedTelemetryProperties = zooManager.getModelLoadedTelemetryProperties(responseJson);
        }
    }

    if(useCachedModel) {
        if(!internetIsAvailable) {
            fs::path modelPath = zooManager.loadModelFromCache();
            logger::info("Using cached model located at {}", modelPath);
            return modelPath;
        }

        auto responseHash = responseJson["hash"].get<std::string>();
        auto metadata = utility::loadYaml(zooManager.getMetadataFilePath());
        auto metadataHash = utility::yamlGet<std::string>(metadata, "hash");

        if(responseHash == metadataHash) {
            fs::path modelPath = zooManager.loadModelFromCache();
            logger::info("Using cached model located at {}", modelPath);
            emitZooModelLoadedTelemetry(std::move(modelLoadedTelemetryProperties), true);
            return modelPath;
        }

        logger::warn("Cached model hash does not match response hash, downloading anew ...");
    }

    if(modelIsCached) {
        logger::info("Removing cached model folder");
        zooManager.removeModelCacheFolder();
    }

    if(!performInternetCheck) {
        throw std::runtime_error("Model no available. Please set DEPTHAI_ZOO_INTERNET_CHECK to 1 to download models from the model zoo.");
    }

    if(performInternetCheck && !internetIsAvailable) {
        throw std::runtime_error("Model not available. No internet connection available. Could not connect to host: " + dai::modelzoo::getHealthEndpoint());
    }

    zooManager.createFolder(zooManager.getModelCacheFolderName());

    std::unique_ptr<CprCallback> cprCallback =
        getCprCallback(progressFormat, modelDescription.globalMetadataEntryName.size() > 0 ? modelDescription.globalMetadataEntryName : modelDescription.model);

    logger::info("Downloading model from model zoo");
    zooManager.downloadModel(responseJson, cprCallback->getCprProgressCallback());

    fs::path yamlPath = zooManager.getModelCacheFolderPath(cacheDirectory) / "model.yaml";
    modelDescription.saveToYamlFile(yamlPath);

    fs::path modelPath = zooManager.loadModelFromCache();
    emitZooModelLoadedTelemetry(std::move(modelLoadedTelemetryProperties), false);
    return modelPath;
}

bool downloadModelsFromZoo(const fs::path& path, const fs::path& cacheDirectory, const std::string& apiKey, const std::string& progressFormat) {
    logger::info("Downloading models from zoo");
    if(!std::filesystem::exists(path)) throw std::runtime_error(fmt::format("Path does not exist: {}", path));

    std::vector<fs::path> models;
    for(const auto& entry : std::filesystem::recursive_directory_iterator(path)) {
        fs::path modelPath = std::filesystem::path(entry).lexically_relative(path).lexically_normal();
        if(utility::isYamlFile(modelPath)) {
            logger::debug("Found model in folder {}: {}", path, modelPath);
            models.push_back(modelPath);
        }
    }

    int numSuccess = 0, numFail = 0;
    for(size_t i = 0; i < models.size(); ++i) {
        const fs::path& modelName = models[i];

        try {
            logger::info("Downloading model [{} / {}]: {}", i + 1, models.size(), modelName);
            auto modelDescription = NNModelDescription::fromYamlFile(modelName, path);
            getModelFromZoo(modelDescription, true, cacheDirectory, apiKey, progressFormat);
            logger::info("Downloaded model [{} / {}]: {}", i + 1, models.size(), modelName);
            numSuccess++;
        } catch(const std::exception& e) {
            const std::string errorMessage =
                fmt::format("Failed to download model [{} / {}]: {} in folder {}\n{}", i + 1, models.size(), modelName, path, e.what());
            if(progressFormat == "json") {
                std::cout << nlohmann::json({{"log_type", "error"}, {"model_name", modelName}, {"detail", errorMessage}}).dump() << std::endl;
            } else {
                logger::error(errorMessage);
            }
            numFail++;
        }
    }

    logger::info("Downloaded {} models from folder {} | {} failed.", numSuccess, path, numFail);
    return numFail == 0;
}

bool ZooManager::connectionToZooAvailable() {
    int timeoutMs = utility::getEnvAs<int>("DEPTHAI_ZOO_INTERNET_CHECK_TIMEOUT", 1000);  // default is 1000ms
    const std::string host = dai::modelzoo::getHealthEndpoint();

    bool connected = false;
    try {
        cpr::Response r = cpr::Get(cpr::Url{host}, cpr::Timeout{timeoutMs});
        connected = r.status_code == cpr::status::HTTP_OK;
    } catch(const cpr::Error& e) {
    }

    return connected;
}

fs::path ZooManager::getMetadataFilePath() const {
    return getModelCacheFolderPath(cacheDirectory) / "metadata.yaml";
}

fs::path ZooManager::getGlobalMetadataFilePath() const {
    return cacheDirectory / "metadata.yaml";
}

void ZooManager::createFolder(const std::string& folderName) const {
    auto folderPath = platform::joinPaths(cacheDirectory, folderName);
    std::filesystem::create_directories(folderPath);
}

#else

fs::path getModelFromZoo(
    const NNModelDescription& modelDescription, bool useCached, const fs::path& cacheDirectory, const std::string& apiKey, const std::string& progressFormat) {
    (void)modelDescription;
    (void)useCached;
    (void)cacheDirectory;
    (void)apiKey;
    (void)progressFormat;
    throw std::runtime_error("getModelFromZoo requires libcurl to be enabled. Please recompile DepthAI with libcurl enabled.");
}

bool downloadModelsFromZoo(const fs::path& path, const fs::path& cacheDirectory, const std::string& apiKey, const std::string& progressFormat) {
    (void)path;
    (void)cacheDirectory;
    (void)apiKey;
    (void)progressFormat;
    throw std::runtime_error("downloadModelsFromZoo requires libcurl to be enabled. Please recompile DepthAI with libcurl enabled.");
}

#endif

fs::path getYamlFilePath(const fs::path& name, const fs::path& modelsPath) {
    if(name.empty()) throw std::runtime_error("name cannot be empty!");

    auto _check = [](const fs::path::string_type& c) {
#if defined(WIN32) || defined(_WIN32)
        return std::iswalpha(c[0]) || std::iswdigit(c[0]);
#else
        return std::isalpha(c[0]) || std::isdigit(c[0]);
#endif
    };

    if(_check(name.native())) {
        fs::path useModelsPath = modelsPath;
        if(useModelsPath.empty()) {
            useModelsPath = fs::path(utility::getEnvAs<fs::path>("DEPTHAI_ZOO_MODELS_PATH", dai::modelzoo::getDefaultModelsPath(), false));
        }

        fs::path path = useModelsPath / name;

        if(!utility::isYamlFile(path) && !path.has_extension()) {
            fs::path yamlPath = path.replace_extension(".yaml");
            if(std::filesystem::exists(yamlPath)) {
                return yamlPath;
            }

            fs::path ymlPath = path.replace_extension(".yml");
            if(std::filesystem::exists(ymlPath)) {
                return ymlPath;
            }

            throw std::runtime_error(fmt::format(
                "Model file not found: (neither `{}` nor `{}` exists) | If you meant to use a relative path, prefix it with ./ (e.g. `./{}`) | Also, "
                "make sure the file exists. Read the documentation for more information.",
                yamlPath,
                ymlPath,
                name));
        }
        return path;
    }

    return name;
}

std::string SlugComponents::merge() const {
    std::ostringstream oss;
    if(!teamName.empty()) {
        oss << teamName << "/";
    }
    oss << modelSlug;
    if(!modelVariantSlug.empty()) {
        oss << ":" << modelVariantSlug;
    }
    if(!modelRef.empty()) {
        oss << ":" << modelRef;
    }
    return oss.str();
}

SlugComponents SlugComponents::split(const std::string& slug) {
    SlugComponents components;
    std::istringstream iss(slug);
    std::string part;
    int partIndex = 0;

    while(std::getline(iss, part, ':')) {
        if(partIndex == 0) {
            auto slashPos = part.find('/');
            if(slashPos != std::string::npos) {
                components.teamName = part.substr(0, slashPos);
                components.modelSlug = part.substr(slashPos + 1);
            } else {
                components.modelSlug = part;
            }
        } else if(partIndex == 1) {
            components.modelVariantSlug = part;
        } else if(partIndex == 2) {
            components.modelRef = part;
        }
        partIndex++;
    }

    return components;
}

NNModelDescription NNModelDescription::fromYamlFile(const fs::path& modelName, const fs::path& modelsPath) {
    fs::path yamlPath = getYamlFilePath(modelName, modelsPath);
    if(!std::filesystem::exists(yamlPath)) {
        throw std::runtime_error(
            fmt::format("Model file not found: `{}` | If you meant to use a relative path, prefix with ./ (e.g. `./{}`) | Also, "
                        "make sure the file exists. Read the documentation for more information.",
                        yamlPath,
                        modelName));
    }

    auto yamlNode = utility::loadYaml(yamlPath);

    auto model = utility::yamlGet<std::string>(yamlNode, "model");

    auto platform = utility::yamlGet<std::string>(yamlNode, "platform", "");
    auto optimizationLevel = utility::yamlGet<std::string>(yamlNode, "optimization_level", "");
    auto compressionLevel = utility::yamlGet<std::string>(yamlNode, "compression_level", "");
    auto snpeVersion = utility::yamlGet<std::string>(yamlNode, "snpe_version", "");
    auto modelPrecisionType = utility::yamlGet<std::string>(yamlNode, "model_precision_type", "");

    auto globalMetadataEntryName = modelName.string();

    return {model, platform, optimizationLevel, compressionLevel, snpeVersion, modelPrecisionType, globalMetadataEntryName};
}

void NNModelDescription::saveToYamlFile(const fs::path& yamlPath) const {
    YAML::Node yamlNode;

    yamlNode["model"] = model;

    if(!platform.empty()) yamlNode["platform"] = platform;
    if(!optimizationLevel.empty()) yamlNode["optimization_level"] = optimizationLevel;
    if(!compressionLevel.empty()) yamlNode["compression_level"] = compressionLevel;
    if(!snpeVersion.empty()) yamlNode["snpe_version"] = snpeVersion;
    if(!modelPrecisionType.empty()) yamlNode["model_precision_type"] = modelPrecisionType;

    utility::saveYaml(yamlNode, yamlPath);
}

bool NNModelDescription::check() const {
    return !model.empty() && !platform.empty();
}

std::string NNModelDescription::toString() const {
    std::string out = "NNModelDescription [\n";
    out += "  model: " + model + "\n";
    out += "  platform: " + platform + "\n";
    out += "  optimization_level: " + optimizationLevel + "\n";
    out += "  compression_level: " + compressionLevel + "\n";
    out += "  snpe_version: " + snpeVersion + "\n";
    out += "  model_precision_type: " + modelPrecisionType + "\n";
    out += "]";
    return out;
}

std::ostream& operator<<(std::ostream& os, const NNModelDescription& modelDescription) {
    os << modelDescription.toString();
    return os;
}

namespace modelzoo {

void setHealthEndpoint(const std::string& endpoint) {
    const std::lock_guard<std::mutex> lock(modelZooSettingsMutex);
    modelZooHealthEndpoint = endpoint;
}

void setDownloadEndpoint(const std::string& endpoint) {
    const std::lock_guard<std::mutex> lock(modelZooSettingsMutex);
    modelZooDownloadEndpoint = endpoint;
}

void setDefaultCachePath(const fs::path& path) {
    const std::lock_guard<std::mutex> lock(modelZooSettingsMutex);
    modelZooDefaultCachePath = path;
}

void setDefaultModelsPath(const fs::path& path) {
    const std::lock_guard<std::mutex> lock(modelZooSettingsMutex);
    modelZooDefaultModelsPath = path;
}

std::string getHealthEndpoint() {
    const std::lock_guard<std::mutex> lock(modelZooSettingsMutex);
    return modelZooHealthEndpoint;
}

std::string getDownloadEndpoint() {
    const std::lock_guard<std::mutex> lock(modelZooSettingsMutex);
    return modelZooDownloadEndpoint;
}

fs::path getDefaultCachePath() {
    const std::lock_guard<std::mutex> lock(modelZooSettingsMutex);
    return modelZooDefaultCachePath;
}

fs::path getDefaultModelsPath() {
    const std::lock_guard<std::mutex> lock(modelZooSettingsMutex);
    return modelZooDefaultModelsPath;
}

}  // namespace modelzoo

}  // namespace dai
