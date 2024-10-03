#pragma once

#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>

#include "depthai/pipeline/datatype/ADatatype.hpp"
#include "depthai/schemas/Event.pb.h"

namespace dai {
namespace utility {
struct FileData {
    std::vector<char> data;
    std::string fileName;
    std::string fileUrl;
    std::uintmax_t fileSize;
    std::string mimeType;
};
struct EventMessage {
    std::unique_ptr<proto::Event> event;
    std::vector<FileData> files;
};
class EventsManager {
   public:
    explicit EventsManager(const std::string& deviceSerialNumber);
    virtual ~EventsManager() = default;

    void sendEvent(const std::string& name,
                   const std::unordered_map<std::string, std::string>& data,
                   const std::vector<std::string>& tags,
                   const std::vector<FileData>& files = {},
                   const std::shared_ptr<ADatatype>& daiMsg = nullptr);
    void sendSnap(const std::string& name,
                  const std::unordered_map<std::string, std::string>& data,
                  const std::vector<std::string>& tags,
                  const std::vector<FileData>& files = {},
                  const std::shared_ptr<ADatatype>& daiMsg = nullptr);
    void setUrl(const std::string& url);
    void setSourceAppId(const std::string& sourceAppId);
    void setSourceAppIdentifier(const std::string& sourceAppIdentifier);
    void setToken(const std::string& token);
    void setQueueSize(unsigned long queuSize);
    void setSendFrequency(float sendFrequency);

   private:
    std::string createUUID();
    void sendEventBuffer();
    std::string token;
    std::string deviceSerialNumber;
    std::string url;
    std::string sourceAppId;
    std::string sourceAppIdentifier;
    unsigned long queueSize;
    std::thread eventBufferThread;
    std::vector<std::unique_ptr<EventMessage>> eventBuffer;
    std::mutex eventBufferMutex;
    float sendFrequency = 1.0f;
};
}  // namespace utility
}  // namespace dai
