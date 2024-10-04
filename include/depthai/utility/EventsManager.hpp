#pragma once

#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>

#include "depthai/pipeline/datatype/ADatatype.hpp"
#include "depthai/schemas/Event.pb.h"
#include "depthai/pipeline/datatype/ImgFrame.hpp"
#include "depthai/pipeline/datatype/NNData.hpp"

namespace dai {
namespace utility {
enum class EventDataType { DATA, FILE_URL, IMG_FRAME, NN_DATA };
class EventData {
public:
	EventData(const std::string& data, const std::string& fileName, const std::string& mimeType);
	explicit EventData(const std::string& fileUrl);
	explicit EventData(const std::shared_ptr<ImgFrame>& imgFrame, const std::string& fileName);
	explicit EventData(const std::shared_ptr<NNData>& nnData, const std::string& fileName);

private:
    std::string fileName;
    std::string mimeType;
	std::string data;
	EventDataType type;
	friend class EventsManager;
};
struct EventMessage {
    std::shared_ptr<proto::Event> event;
    std::vector<std::shared_ptr<EventData>> data;
};
class EventsManager {
   public:
    explicit EventsManager(const std::string& deviceSerialNumber);
    virtual ~EventsManager() = default;

    void sendEvent(const std::string& name,
				   const std::shared_ptr<ImgFrame>& imgFrame = nullptr,
                   std::vector<std::shared_ptr<EventData>> data = {},
                   const std::vector<std::string>& tags = {},
                   const std::unordered_map<std::string, std::string>& extraData = {}
				   );
    void sendSnap(const std::string& name,
				   const std::shared_ptr<ImgFrame>& imgFrame = nullptr,
                   std::vector<std::shared_ptr<EventData>> data = {},
                   const std::vector<std::string>& tags = {},
                   const std::unordered_map<std::string, std::string>& extraData = {}
				   );


    void setUrl(const std::string& url);
    void setSourceAppId(const std::string& sourceAppId);
    void setSourceAppIdentifier(const std::string& sourceAppIdentifier);
    void setToken(const std::string& token);
    void setQueueSize(unsigned long queuSize);
    void setPublishInterval(float publishInterval);
	void setLogResponse(bool logResponse);

   private:
    std::string createUUID();
    void sendEventBuffer();
	void sendFile(std::shared_ptr<EventData> file, const std::string& url);
    std::string token;
    std::string deviceSerialNumber;
    std::string url;
    std::string sourceAppId;
    std::string sourceAppIdentifier;
    unsigned long queueSize;
    std::thread eventBufferThread;
    std::vector<std::shared_ptr<EventMessage>> eventBuffer;
    std::mutex eventBufferMutex;
    float publishInterval;
	bool logResponse;
};
}  // namespace utility
}  // namespace dai
