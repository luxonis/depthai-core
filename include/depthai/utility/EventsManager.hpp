#pragma once

#include <condition_variable>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>
#include <atomic>

#include "depthai/pipeline/datatype/ADatatype.hpp"
#include "depthai/pipeline/datatype/EncodedFrame.hpp"
#include "depthai/pipeline/datatype/ImgFrame.hpp"
#include "depthai/pipeline/datatype/NNData.hpp"

namespace dai {
namespace proto {
namespace event {
class Event;
}  // namespace event
}  // namespace proto
namespace utility {
enum class EventDataType { DATA, FILE_URL, IMG_FRAME, ENCODED_FRAME, NN_DATA };
class EventData {
   public:
    EventData(const std::string& data, const std::string& fileName, const std::string& mimeType);
    explicit EventData(std::string fileUrl);
    explicit EventData(const std::shared_ptr<ImgFrame>& imgFrame, std::string fileName);
    explicit EventData(const std::shared_ptr<EncodedFrame>& encodedFrame, std::string fileName);
    explicit EventData(const std::shared_ptr<NNData>& nnData, std::string fileName);
    bool toFile(const std::string& path);

   private:
    std::string fileName;
    std::string mimeType;
    std::string data;
    EventDataType type;
    friend class EventsManager;
};
class EventsManager {
   public:
    explicit EventsManager(std::string url = "https://events-ingest.cloud.luxonis.com", bool uploadCachedOnStart = false, float publishInterval = 10.0);
    ~EventsManager();

    /**
     * Send an event to the events service
     * @param name Name of the event
     * @param imgFrame Image frame to send
     * @param data List of EventData objects to send
     * @param tags List of tags to send
     * @param extraData Extra data to send
     * @param deviceSerialNo Device serial number
     * @return bool
     */
    bool sendEvent(const std::string& name,
                   const std::shared_ptr<ImgFrame>& imgFrame = nullptr,
                   std::vector<std::shared_ptr<EventData>> data = {},
                   const std::vector<std::string>& tags = {},
                   const std::unordered_map<std::string, std::string>& extraData = {},
                   const std::string& deviceSerialNo = "");
    /**
     * Send a snap to the events service. Snaps should be used for sending images and other large files.
     * @param name Name of the snap
     * @param imgFrame Image frame to send
     * @param data List of EventData objects to send
     * @param tags List of tags to send
     * @param extraData Extra data to send
     * @param deviceSerialNo Device serial number
     * @return bool
     */
    bool sendSnap(const std::string& name,
                  const std::shared_ptr<ImgFrame>& imgFrame = nullptr,
                  std::vector<std::shared_ptr<EventData>> data = {},
                  const std::vector<std::string>& tags = {},
                  const std::unordered_map<std::string, std::string>& extraData = {},
                  const std::string& deviceSerialNo = "");

    void setDeviceSerialNumber(const std::string& deviceSerialNumber);
    /**
     * Set the URL of the events service. By default, the URL is set to https://events-ingest.cloud.luxonis.com
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
     * Check if the device is connected to Hub. Performs a simple GET request to the URL/health endpoint
     * @return bool
     */
    bool checkConnection();

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
    struct EventMessage {
        std::shared_ptr<proto::event::Event> event;
        std::vector<std::shared_ptr<EventData>> data;
        std::string cachePath;
    };
    static std::string createUUID();
    void sendEventBuffer();
    void sendFile(const std::shared_ptr<EventData>& file, const std::string& url);
    void cacheEvents();
    bool checkForCachedData();
    std::string token;
    std::string deviceSerialNumber;
    std::string url;
    std::string sourceAppId;
    std::string sourceAppIdentifier;
    uint64_t queueSize;
    std::unique_ptr<std::thread> eventBufferThread;
    std::vector<std::shared_ptr<EventMessage>> eventBuffer;
    std::mutex eventBufferMutex;
    float publishInterval;
    bool logResponse;
    bool verifySsl;
    bool connected;
    std::string cacheDir;
    bool uploadCachedOnStart;
    bool cacheIfCannotSend;
    std::atomic<bool> stopEventBuffer;
    std::condition_variable eventBufferCondition;
    std::mutex eventBufferConditionMutex;
};
}  // namespace utility
}  // namespace dai
