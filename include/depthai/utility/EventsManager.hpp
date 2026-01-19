#pragma once

#include <atomic>
#include <condition_variable>
#include <future>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>

#include "depthai/pipeline/datatype/ADatatype.hpp"
#include "depthai/pipeline/datatype/EncodedFrame.hpp"
#include "depthai/pipeline/datatype/ImgDetections.hpp"
#include "depthai/pipeline/datatype/ImgFrame.hpp"
#include "depthai/pipeline/datatype/NNData.hpp"

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
    explicit FileData(std::filesystem::path filePath, std::string fileName);
    explicit FileData(const std::shared_ptr<ImgFrame>& imgFrame, std::string fileName);
    explicit FileData(const std::shared_ptr<EncodedFrame>& encodedFrame, std::string fileName);
    // explicit FileData(const std::shared_ptr<NNData>& nnData, std::string fileName);
    explicit FileData(const std::shared_ptr<ImgDetections>& imgDetections, std::string fileName);
    bool toFile(const std::filesystem::path& inputPath);

   private:
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
    void addFile(std::string fileName, std::string data, std::string mimeType);
    void addFile(std::string fileName, std::filesystem::path filePath);
    void addFile(const std::optional<std::string>& fileName, const std::shared_ptr<ImgFrame>& imgFrame);
    void addFile(const std::optional<std::string>& fileName, const std::shared_ptr<EncodedFrame>& encodedFrame);
    // void addFile(std::string fileName, const std::shared_ptr<NNData>& nnData);
    void addFile(const std::optional<std::string>& fileName, const std::shared_ptr<ImgDetections>& imgDetections);
    void addImageDetectionsPair(const std::optional<std::string>& fileName,
                                const std::shared_ptr<ImgFrame>& imgFrame,
                                const std::shared_ptr<ImgDetections>& imgDetections);
    void addImageDetectionsPair(const std::optional<std::string>& fileName,
                                const std::shared_ptr<EncodedFrame>& encodedFrame,
                                const std::shared_ptr<ImgDetections>& imgDetections);
    // void addImageNNDataPair(std::string fileName, const std::shared_ptr<ImgFrame>& imgFrame, const std::shared_ptr<NNData>& imgDetections);
    // void addImageNNDataPair(std::string fileName, const std::shared_ptr<EncodedFrame>& encodedFrame, const std::shared_ptr<NNData>& imgDetections);

   private:
    std::vector<std::shared_ptr<FileData>> fileData;
    friend class EventsManager;
};

enum class SendSnapCallbackStatus {
    SUCCESS,
    FILE_BATCH_PREPARATION_FAILED,
    GROUP_CONTAINS_REJECTED_FILES,
    FILE_UPLOAD_FAILED,
    SEND_EVENT_FAILED,
    EVENT_REJECTED
};

struct SendSnapCallbackResult {
   public:
    std::string snapName;
    int64_t snapTimestamp;
    std::string snapLocalID;
    std::optional<std::string> snapHubID;
    std::string snapPayload;
    SendSnapCallbackStatus uploadStatus;
};

class EventsManager {
   public:
    explicit EventsManager(bool uploadCachedOnStart = false);
    ~EventsManager();

    /**
     * Send an event to the events service
     * @param name Name of the event
     * @param tags List of tags to send
     * @param extras Extra data to send
     * @param associateFiles List of associate files with ids
     * @return LocalID of the sent Event
     */
    std::optional<std::string> sendEvent(const std::string& name,
                                         const std::vector<std::string>& tags = {},
                                         const std::unordered_map<std::string, std::string>& extras = {},
                                         const std::vector<std::string>& associateFiles = {});
    /**
     * Send a snap to the events service. Snaps should be used for sending images and other files.
     * @param name Name of the snap
     * @param fileGroup FileGroup containing FileData objects to send
     * @param tags List of tags to send
     * @param extras Extra data to send
     * @param successCallback Callback to be called when the snap is successfully uploaded to the hub
     * @param failureCallback Callback to be called if the snap upload is unsuccessful
     * @return LocalID of the sent Snap
     */
    std::optional<std::string> sendSnap(const std::string& name,
                                        const std::shared_ptr<FileGroup> fileGroup,
                                        const std::vector<std::string>& tags = {},
                                        const std::unordered_map<std::string, std::string>& extras = {},
                                        const std::function<void(SendSnapCallbackResult)> successCallback = nullptr,
                                        const std::function<void(SendSnapCallbackResult)> failureCallback = nullptr);
    /**
     * Send a snap to the events service, with an ImgFrame and ImgDetections pair as files
     * @param name Name of the snap
     * @param fileName File name used to create FileData
     * @param imgFrame ImgFrame to send
     * @param imgDetections ImgDetections to sent
     * @param tags List of tags to send
     * @param extras Extra data to send
     * @param successCallback Callback to be called when the snap is successfully uploaded to the hub
     * @param failureCallback Callback to be called if the snap upload is unsuccessful
     * @return LocalID of the sent Snap
     */
    std::optional<std::string> sendSnap(const std::string& name,
                                        const std::optional<std::string>& fileName,
                                        const std::shared_ptr<ImgFrame> imgFrame,
                                        const std::optional<std::shared_ptr<ImgDetections>>& imgDetections = std::nullopt,
                                        const std::vector<std::string>& tags = {},
                                        const std::unordered_map<std::string, std::string>& extras = {},
                                        const std::function<void(SendSnapCallbackResult)> successCallback = nullptr,
                                        const std::function<void(SendSnapCallbackResult)> failureCallback = nullptr);
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
    struct EventData {
        std::string localID;
        std::shared_ptr<proto::event::Event> event;
        std::optional<std::function<void(SendSnapCallbackResult)>> onSuccess;
        std::optional<std::function<void(SendSnapCallbackResult)>> onFailure;
    };

    struct SnapData {
        std::shared_ptr<EventData> eventData;
        std::shared_ptr<FileGroup> fileGroup;
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
     * Cache events from the eventBuffer to the filesystem
     */
    void cacheEvents();
    /**
     * Cache snapData from the inputSnapBatch to the filesystem
     */
    void cacheSnapData(std::deque<std::shared_ptr<SnapData>>& inputSnapBatch);
    /**
     * Upload cached data to the events service
     * @return void
     */
    void uploadCachedData();
    /**
     * Check if there's any cached data in the filesystem
     */
    bool checkForCachedData();
    /**
     * Clear cached data in the filesystem (if any)
     */
    void clearCachedData(const std::filesystem::path& directory);

    std::string token;
    std::string url;
    std::string sourceAppId;
    std::string sourceAppIdentifier;
    std::string sourceSerialNumber;
    float publishInterval;
    bool logResponse;
    bool verifySsl;
    std::string cacheDir;
    bool cacheIfCannotSend;
    std::unique_ptr<std::thread> uploadThread;
    std::deque<std::shared_ptr<EventData>> eventBuffer;
    std::deque<std::shared_ptr<SnapData>> snapBuffer;
    std::deque<std::future<void>> uploadFileBatchFutures;
    std::mutex eventBufferMutex;
    std::mutex snapBufferMutex;
    std::mutex stopThreadConditionMutex;
    std::atomic<bool> stopUploadThread;
    std::atomic<bool> configurationLimitsFetched;
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

    static constexpr int EVENT_BUFFER_MAX_SIZE = 300;

    static constexpr int EVENT_VALIDATION_NAME_LENGTH = 56;
    static constexpr int EVENT_VALIDATION_MAX_TAGS = 20;
    static constexpr int EVENT_VALIDATION_TAG_LENGTH = 56;
    static constexpr int EVENT_VALIDATION_MAX_EXTRAS = 25;
    static constexpr int EVENT_VALIDATION_EXTRA_KEY_LENGTH = 40;
    static constexpr int EVENT_VALIDATION_EXTRA_VALUE_LENGTH = 100;
    static constexpr int EVENT_VALIDATION_MAX_ASSOCIATE_FILES = 20;
};
}  // namespace utility
}  // namespace dai
