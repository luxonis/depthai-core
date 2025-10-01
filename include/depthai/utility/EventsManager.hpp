#pragma once

#include <atomic>
#include <condition_variable>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>

#include "depthai/pipeline/datatype/ADatatype.hpp"
#include "depthai/pipeline/datatype/EncodedFrame.hpp"
#include "depthai/pipeline/datatype/ImgFrame.hpp"
#include "depthai/pipeline/datatype/NNData.hpp"
#include "depthai/pipeline/datatype/ImgDetections.hpp"

namespace dai {
namespace proto {
namespace event {
class Event;
enum PrepareFileUploadClass : int;
}  // namespace event
}  // namespace proto

namespace utility {

class FileData {
   public:
    FileData(const std::string& data, const std::string& fileName, const std::string& mimeType);
    explicit FileData(const std::string& filePath, std::string fileName);
    explicit FileData(const std::shared_ptr<ImgFrame>& imgFrame, std::string fileName);
    explicit FileData(const std::shared_ptr<EncodedFrame>& encodedFrame, std::string fileName);
    explicit FileData(const std::shared_ptr<NNData>& nnData, std::string fileName);
    explicit FileData(const std::shared_ptr<ImgDetections>& imgDetections, std::string fileName);
    bool toFile(const std::string& inputPath);

   private:
    /**
     * Calculate SHA256 checksum for the given data
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
     * @param fileGroup List of FileData objects to send
     * @return bool
     */
    bool sendSnap(const std::string& name,
                  const std::vector<std::string>& tags = {},
                  const std::unordered_map<std::string, std::string>& extras = {},
                  const std::string& deviceSerialNo = "",
                  const std::vector<std::shared_ptr<FileData>>& fileGroup = {});

    void setDeviceSerialNumber(const std::string& deviceSerialNumber);
    /**
     * Set the URL of the events service. By default, the URL is set to https://events.cloud.luxonis.com
     * @param url URL of the events service
     * @return void
     */
    void setUrl(const std::string& url);
    /**
     * Set the source app ID. By default, the source app ID is taken from the environment variable AGENT_APP_ID
     * @param sourceAppId Source app ID
     * @return void
     */
    void setSourceAppId(const std::string& sourceAppId);
    /**
     * Set the source app identifier. By default, the source app identifier is taken from the environment variable AGENT_APP_IDENTIFIER
     * @param sourceAppIdentifier Source app identifier
     * @return void
     */
    void setSourceAppIdentifier(const std::string& sourceAppIdentifier);
    /**
     * Set the token for the events service. By default, the token is taken from the environment variable DEPTHAI_HUB_API_KEY
     * @param token Token for the events service
     * @return void
     */
    void setToken(const std::string& token);
    /**
     * Set the queue size for the amount of events that can be added and sent. By default, the queue size is set to 10
     * @param queueSize Queue size
     * @return void
     */
    void setQueueSize(uint64_t queuSize);
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
        std::vector<std::shared_ptr<FileData>> fileGroup;
        std::string cachePath;
    };

    /**
     * Prepare and upload files from snapBuffer in batch
     */
    void uploadFileBatch();
    /**
     * Upload events from eventBuffer in batch
     */
    void uploadEventBatch();
    /**
     * Upload a file using the chosen url
     */
    void uploadFile(const std::shared_ptr<FileData>& file, const std::string& url);
    /**
     * // TO DO: Add description
     */
    void cacheEvents();
    /**
     * // TO DO: Add description
     */
    bool checkForCachedData();
    /**
     * Validate the input event by checking that its fields adhere to defined limitations
     * @param inputEvent Input event to be validated
     * @return bool
     */
    bool validateEvent(const proto::event::Event& inputEvent);
    /**
     * Fetch configuration limits and quotas for snaps, through the api
     * @return bool
     */
    bool fetchConfigurationLimits();

    std::string token;
    std::string deviceSerialNumber;
    std::string url;
    std::string sourceAppId;
    std::string sourceAppIdentifier;
    uint64_t queueSize;
    float publishInterval;
    bool logResponse;
    bool verifySsl;
    std::string cacheDir;
    bool cacheIfCannotSend;
    std::unique_ptr<std::thread> uploadThread;
    std::vector<std::shared_ptr<proto::event::Event>> eventBuffer;
    std::vector<std::shared_ptr<SnapData>> snapBuffer;
    std::mutex eventBufferMutex;
    std::mutex snapBufferMutex;
    std::mutex stopThreadConditionMutex;
    std::atomic<bool> stopUploadThread;
    std::condition_variable eventBufferCondition;

    uint64_t maximumFileSize;
    uint64_t remainingStorage;
    uint64_t warningStorage;
    uint64_t bytesPerHour;
    uint32_t uploadsPerHour;
    uint32_t eventsPerHour;
    uint32_t snapsPerHour;

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
