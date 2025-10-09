#pragma once

#include <atomic>
#include <condition_variable>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>
#include <future>

#include "depthai/pipeline/datatype/ADatatype.hpp"
#include "depthai/pipeline/datatype/EncodedFrame.hpp"
#include "depthai/pipeline/datatype/ImgFrame.hpp"
#include "depthai/pipeline/datatype/NNData.hpp"
#include "depthai/pipeline/datatype/ImgDetections.hpp"

namespace dai {
namespace proto {
namespace event {
class Event;
class FileUploadGroupResult;
enum PrepareFileUploadClass : int;
}  // namespace event
}  // namespace proto

namespace utility {

class FileData {
   public:
    FileData(std::string data, std::string fileName, std::string mimeType);
    explicit FileData(std::string filePath, std::string fileName);
    explicit FileData(const std::shared_ptr<ImgFrame>& imgFrame, std::string fileName);
    explicit FileData(const std::shared_ptr<EncodedFrame>& encodedFrame, std::string fileName);
    explicit FileData(const std::shared_ptr<NNData>& nnData, std::string fileName);
    explicit FileData(const std::shared_ptr<ImgDetections>& imgDetections, std::string fileName);
    bool toFile(const std::string& inputPath);

   private:
    /**
     * Calculate SHA256 checksum for the given data
     * @param data Data for checksum calculation
     * @return checksum string
     */
    std::string calculateSHA256Checksum(const std::string& data);

    std::string mimeType;
    std::string fileName;
    std::string data;
    uint64_t size;
    std::string checksum;
    proto::event::PrepareFileUploadClass classification;
    friend class EventsManager;
};

class FileGroup {
   public:
    void addFile(std::string data, std::string fileName, std::string mimeType);
    void addFile(std::string filePath, std::string fileName);
    void addFile(const std::shared_ptr<ImgFrame>& imgFrame, std::string fileName);
    void addFile(const std::shared_ptr<EncodedFrame>& encodedFrame, std::string fileName);
    void addFile(const std::shared_ptr<NNData>& nnData, std::string fileName);
    void addFile(const std::shared_ptr<ImgDetections>& imgDetections, std::string fileName);
    void addImageDetectionsPair(const std::shared_ptr<ImgFrame>& imgFrame, 
                                const std::shared_ptr<ImgDetections>& imgDetections, 
                                std::string fileName);
    void addImageDetectionsPair(const std::shared_ptr<EncodedFrame>& encodedFrame, 
                                const std::shared_ptr<ImgDetections>& imgDetections, 
                                std::string fileName);

   private:
    std::vector<std::shared_ptr<FileData>> fileData;
    friend class EventsManager;
};

class EventsManager {
   public:
    explicit EventsManager(std::string url = "https://events.cloud.luxonis.com", bool uploadCachedOnStart = false, float publishInterval = 10.0);
    ~EventsManager();

    /**
     * Send an event to the events service
     * @param name Name of the event
     * @param tags List of tags to send
     * @param extras Extra data to send
     * @param deviceSerialNo Device serial number
     * @param associateFiles List of associate files with ids
     * @return bool
     */
    bool sendEvent(const std::string& name,
                   const std::vector<std::string>& tags = {},
                   const std::unordered_map<std::string, std::string>& extras = {},
                   const std::string& deviceSerialNo = "",
                   const std::vector<std::string>& associateFiles = {});
    /**
     * Send a snap to the events service. Snaps should be used for sending images and other files.
     * @param name Name of the snap
     * @param tags List of tags to send
     * @param extras Extra data to send
     * @param deviceSerialNo Device serial number
     * @param fileGroup FileGroup containing FileData objects to send
     * @return bool
     */
    bool sendSnap(const std::string& name,
                  const std::vector<std::string>& tags = {},
                  const std::unordered_map<std::string, std::string>& extras = {},
                  const std::string& deviceSerialNo = "",
                  const std::shared_ptr<FileGroup> fileGroup = nullptr);

    /**
     * Set the URL of the events service. By default, the URL is set to https://events.cloud.luxonis.com
     * @param url URL of the events service
     * @return void
     */
    void setUrl(const std::string& url);
    /**
     * Set the token for the events service. By default, the token is taken from the environment variable DEPTHAI_HUB_API_KEY
     * @param token Token for the events service
     * @return void
     */
    void setToken(const std::string& token);
    /**
     * Set whether to log the responses from the server. By default, logResponse is set to false
     * @param logResponse bool
     * @return void
     */
    void setLogResponse(bool logResponse);
    /**
     * Set whether to verify the SSL certificate. By default, verifySsl is set to false
     * @param verifySsl bool
     * @return void
     */
    void setVerifySsl(bool verifySsl);

    // TO DO: Should be private?
    /**
     * Upload cached data to the events service
     * @return void
     */
    void uploadCachedData();

    /**
     * Set the cache directory for storing cached data. By default, the cache directory is set to /internal/private
     * @param cacheDir Cache directory
     * @return void
     */
    void setCacheDir(const std::string& cacheDir);

    /**
     * Set whether to cache data if it cannot be sent. By default, cacheIfCannotSend is set to false
     * @param cacheIfCannotSend bool
     * @return void
     */
    void setCacheIfCannotSend(bool cacheIfCannotSend);

   private:
    struct SnapData {
        std::shared_ptr<proto::event::Event> event;
        std::shared_ptr<FileGroup> fileGroup;
        std::string cachePath;
    };

    struct UploadRetryPolicy {
        int maxAttempts = 10;
        float factor = 2.0f;
        std::chrono::milliseconds baseDelay{100};
    };

    /**
     * Fetch the configuration limits and quotas for snaps & events
     * @return bool
     */
    bool fetchConfigurationLimits();
    /**
     * Prepare a batch of file groups from inputSnapBatch
     */
    void uploadFileBatch(std::deque<std::shared_ptr<SnapData>> inputSnapBatch);
    /**
     * Upload a prepared group of files from snapData, using prepareGroupResult
     */
    bool uploadGroup(std::shared_ptr<SnapData> snapData, dai::proto::event::FileUploadGroupResult prepareGroupResult);
    /**
     * Upload a file from fileData using the chosen uploadUrl
     */
    bool uploadFile(std::shared_ptr<FileData> fileData, std::string uploadUrl);
    /**
     * Upload events from eventBuffer in batch
     */
    void uploadEventBatch();
    /**
     * Validate the input event by checking that its fields adhere to defined limitations
     * @param inputEvent Input event to be validated
     * @return bool
     */
    bool validateEvent(const proto::event::Event& inputEvent);
    /**
     * // TO DO: Add description
     */
    void cacheEvents();
    /**
     * // TO DO: Add description
     */
    bool checkForCachedData();

    std::string token;
    std::string url;
    std::string sourceAppId;
    std::string sourceAppIdentifier;
    float publishInterval;
    bool logResponse;
    bool verifySsl;
    std::string cacheDir;
    bool cacheIfCannotSend;
    std::unique_ptr<std::thread> uploadThread;
    std::deque<std::shared_ptr<proto::event::Event>> eventBuffer;
    std::deque<std::shared_ptr<SnapData>> snapBuffer;
    std::vector<std::future<void>> uploadFileBatchFutures;
    std::mutex eventBufferMutex;
    std::mutex snapBufferMutex;
    std::mutex stopThreadConditionMutex;
    std::atomic<bool> stopUploadThread;
    std::condition_variable eventBufferCondition;

    uint64_t maxFileSizeBytes;
    uint64_t remainingStorageBytes;
    uint64_t warningStorageBytes;
    uint64_t bytesPerHour;
    uint32_t uploadsPerHour;
    uint32_t maxGroupsPerBatch;
    uint32_t maxFilesPerGroup;
    uint32_t eventsPerHour;
    uint32_t snapsPerHour;
    uint32_t eventsPerRequest;

    UploadRetryPolicy uploadRetryPolicy;

    static constexpr int eventValidationNameLength = 56;
    static constexpr int eventValidationMaxTags = 20;
    static constexpr int eventValidationTagLength = 56;
    static constexpr int eventValidationMaxExtras = 25;
    static constexpr int eventValidationExtraKeyLength = 40;
    static constexpr int eventValidationExtraValueLength = 100;
    static constexpr int eventValidationMaxAssociateFiles = 20;
};
}  // namespace utility
}  // namespace dai
