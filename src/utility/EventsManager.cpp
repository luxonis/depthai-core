#include "depthai/utility/EventsManager.hpp"

#include <chrono>
#include <iostream>
#include <random>
#include <sstream>

#include "Environment.hpp"
#include "Logging.hpp"
#include "depthai/schemas/Event.pb.h"
#ifdef DEPTHAI_ENABLE_CURL
    #include <cpr/cpr.h>
namespace dai {

namespace utility {

EventData::EventData(const std::string& data, const std::string& fileName, const std::string& mimeType)
    : fileName(fileName), mimeType(mimeType), type(EventDataType::DATA) {
    this->data = data;
}


EventData::EventData(const std::string& fileUrl) : data(fileUrl), type(EventDataType::FILE_URL) {}

EventData::EventData(const std::shared_ptr<ImgFrame>& imgFrame, const std::string& fileName) : fileName(fileName), type(EventDataType::IMG_FRAME) {
    // Convert ImgFrame to bytes
    std::stringstream ss;
    ss.write((const char*)imgFrame->data->getData().data(), imgFrame->data->getData().size());
    data = ss.str();
    mimeType = "image/jpeg";
}

EventData::EventData(const std::shared_ptr<NNData>& nnData, const std::string& fileName) : fileName(fileName), type(EventDataType::NN_DATA) {
    // Convert NNData to bytes
    std::stringstream ss;
    ss.write((const char*)nnData->data->getData().data(), nnData->data->getData().size());
    data = ss.str();
    mimeType = "application/octet-stream";
}

EventsManager::EventsManager(const std::string& deviceSerialNumber)
    : deviceSerialNumber(deviceSerialNumber),
      url("https://events-ingest.cloud.luxonis.com/v1/events/"),
      queueSize(10),
      publishInterval(5.0f),
      logResponse(false) {
    sourceAppId = utility::getEnv("AGENT_APP_ID");
    sourceAppIdentifier = utility::getEnv("AGENT_APP_IDENTIFIER");
    token = utility::getEnv("DEPTHAI_HUB_API_KEY");
    eventBufferThread = std::thread([this]() {
        while(true) {
            sendEventBuffer();
            std::this_thread::sleep_for(std::chrono::milliseconds(int(publishInterval * 1000)));
        }
    });
}

void EventsManager::sendEventBuffer() {
    std::lock_guard<std::mutex> lock(eventBufferMutex);
    if(eventBuffer.empty()) {
        return;
    }
    // Create request
    logger::info("Sending events to: {} token: {}", url, token);
    cpr::Url url = cpr::Url(this->url);
    cpr::Body body;
    auto batchEvent = std::make_unique<proto::BatchUploadEvents>();
    for(auto& eventM : eventBuffer) {
        auto& event = eventM->event;
        batchEvent->add_events()->Swap(event.get());
    }
    std::string serializedEvent;
    batchEvent->SerializeToString(&serializedEvent);
    body = cpr::Body(serializedEvent);
    cpr::Response r = cpr::Post(url, cpr::Header{{"Authorization", token}}, cpr::Body(body), cpr::VerifySsl(false));
    if(r.status_code != cpr::status::HTTP_OK) {
        logger::error("Failed to send event: {} {}", r.text, r.status_code);
    } else {
        logger::debug("Event sent successfully");
        if(logResponse) {
            logger::info("Response: {}", r.text);
        }
        // upload files
        auto BatchUploadEventResult = std::make_unique<proto::BatchUploadEventsResult>();
        BatchUploadEventResult->ParseFromString(r.text);
        for(int i = 0; i < BatchUploadEventResult->events_size(); i++) {
            auto eventResult = BatchUploadEventResult->events(i);
            if(eventResult.file_upload_urls().size() > 0) {
                logger::debug("Uploading files");
                for(int j = 0; j < eventResult.file_upload_urls().size(); j++) {
                    logger::debug("Uploading file {}", j);
                    cpr::Url fileUrl = cpr::Url(url + eventResult.file_upload_urls(j));

                    sendFile(eventBuffer[i]->data[j], fileUrl);
                }
            }
        }
        // uncomment to publish files anyway
        // for(auto& eventM : eventBuffer) {
        //     for(auto& file : eventM->files) {
        //         sendFile(file, url);
        //     }
        // }
        eventBuffer.clear();
    }
}

void EventsManager::sendEvent(const std::string& name,
                              const std::shared_ptr<ImgFrame>& imgFrame,
                              std::vector<std::shared_ptr<EventData>> data,
                              const std::vector<std::string>& tags,
                              const std::unordered_map<std::string, std::string>& extraData) {
    // Create event
    auto event = std::make_unique<proto::Event>();
    event->set_nonce(createUUID());
    event->set_name(name);
    event->set_created_at(std::chrono::duration_cast<std::chrono::seconds>(std::chrono::system_clock::now().time_since_epoch()).count());
    for(const auto& tag : tags) {
        event->add_tags(tag);
    }
    proto::Extras* extras = event->add_extras();
    auto* extrasData = extras->mutable_data();
    for(const auto& [key, value] : extraData) {
        extrasData->insert({key, value});
    }

    if(imgFrame != nullptr) {
        auto fileData = std::make_shared<EventData>(imgFrame, "img.jpg");
		data.push_back(fileData);
    }
    event->set_expect_files_num(data.size());

    event->set_source_serial_number(deviceSerialNumber);
    event->set_source_app_id(sourceAppId);
    event->set_source_app_identifier(sourceAppIdentifier);
    // Add event to buffer
    if(eventBuffer.size() <= queueSize) {
        std::lock_guard<std::mutex> lock(eventBufferMutex);
        auto eventMessage = std::make_unique<EventMessage>();
        eventMessage->data = std::move(data);
        eventMessage->event = std::move(event);
        eventBuffer.push_back(std::move(eventMessage));
    } else {
        logger::warn("Event buffer is full, dropping event");
    }
}

void EventsManager::sendSnap(const std::string& name,
                             const std::shared_ptr<ImgFrame>& imgFrame,
                             std::vector<std::shared_ptr<EventData>> data,
                             const std::vector<std::string>& tags,
                             const std::unordered_map<std::string, std::string>& extraData) {
    std::vector<std::string> tagsTmp = tags;
    tagsTmp.push_back("snap");
    // only one image can be in files
    for(const auto& file : data) {
        if(std::any_of(data.begin(), data.end(), [](const std::shared_ptr<EventData>& file) { return file->type == EventDataType::IMG_FRAME; })) {
            logger::error("You can only send one ImgFrame via method argument, sending ImgFrames via files is not supported");
            return;
        }
    }
    return sendEvent(name, imgFrame, data, tagsTmp, extraData);
}

void EventsManager::sendFile(std::shared_ptr<EventData> file, const std::string& url) {
    // if file struct contains byte data, send it, along with filename and mime type
    // if it file url, send it directly via url
    cpr::Multipart fileM{};
    if(!file->data.empty()) {
        fileM = cpr::Multipart{{"file", cpr::Buffer{file->data.begin(), file->data.end(), file->fileName}},
                               {"filename", file->fileName},
                               {"file_size", std::to_string(file->data.size())},
                               {"mime_type", file->mimeType}};
    } else {
        fileM = cpr::Multipart{{
            "file",
            cpr::File{file->data},
        }};
    }
    cpr::Response r = cpr::Post(url, cpr::Header{{"Authorization", "Bearer " + token}}, fileM);
    if(r.status_code != cpr::status::HTTP_OK) {
        logger::error("Failed to upload file: {}", r.text);
    }
    if(logResponse) {
        logger::info("Response: {}", r.text);
    }
}

void EventsManager::setUrl(const std::string& url) {
    this->url = url;
}

void EventsManager::setSourceAppId(const std::string& sourceAppId) {
    this->sourceAppId = sourceAppId;
}

void EventsManager::setSourceAppIdentifier(const std::string& sourceAppIdentifier) {
    this->sourceAppIdentifier = sourceAppIdentifier;
}

void EventsManager::setToken(const std::string& token) {
    this->token = token;
}

std::string EventsManager::createUUID() {
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_int_distribution<> dis(0, 15);
    std::uniform_int_distribution<> dis2(8, 11);

    std::stringstream ss;
    int i;
    ss << std::hex;
    for(i = 0; i < 8; i++) {
        ss << dis(gen);
    }
    ss << "-";
    for(i = 0; i < 4; i++) {
        ss << dis(gen);
    }
    ss << "-4";
    for(i = 0; i < 3; i++) {
        ss << dis(gen);
    }
    ss << "-";
    ss << dis2(gen);
    for(i = 0; i < 3; i++) {
        ss << dis(gen);
    }
    ss << "-";
    for(i = 0; i < 12; i++) {
        ss << dis(gen);
    };
    return ss.str();
}
void EventsManager::setQueueSize(unsigned long queueSize) {
    this->queueSize = queueSize;
}
void EventsManager::setPublishInterval(float publishInterval) {
    this->publishInterval = publishInterval;
}
void EventsManager::setLogResponse(bool logResponse) {
    this->logResponse = logResponse;
}
}  // namespace utility
}  // namespace dai
#else
namespace dai {
namespace utility {
EventsManager::EventsManager(const std::string& deviceSerialNumber) {
    logger::warn("EventsManager is disabled, please enable DEPTHAI_ENABLE_CURL in CMake to use this feature");
}
void EventsManager::sendEvent(const std::string& name,
                              const std::unordered_map<std::string, std::string>& data,
                              const std::vector<std::string>& tags,
                              const std::vector<FileData>& files,
                              const std::shared_ptr<ADatatype>& daiMsg) {
    logger::warn("EventsManager is disabled, please enable DEPTHAI_ENABLE_CURL in CMake to use this feature");
}
void EventsManager::sendSnap(const std::string& name,
                             const std::vector<std::string>& tags,
                             const std::vector<FileData>& files,
                             const std::unordered_map<std::string, std::string>& extraData) {
    logger::warn("EventsManager is disabled, please enable DEPTHAI_ENABLE_CURL in CMake to use this feature");
}
void EventsManager::setUrl(const std::string& url) {
    logger::warn("EventsManager is disabled, please enable DEPTHAI_ENABLE_CURL in CMake to use this feature");
}
void EventsManager::setSourceAppId(const std::string& sourceAppId) {
    logger::warn("EventsManager is disabled, please enable DEPTHAI_ENABLE_CURL in CMake to use this feature");
}
void EventsManager::setSourceAppIdentifier(const std::string& sourceAppIdentifier) {
    logger::warn("EventsManager is disabled, please enable DEPTHAI_ENABLE_CURL in CMake to use this feature");
}
void EventsManager::setToken(const std::string& token) {
    logger::warn("EventsManager is disabled, please enable DEPTHAI_ENABLE_CURL in CMake to use this feature");
}
void EventsManager::setQueueSize(unsigned long queueSize) {
    logger::warn("EventsManager is disabled, please enable DEPTHAI_ENABLE_CURL in CMake to use this feature");
}
void EventsManager::setPublishInterval(float publishInterval) {
    logger::warn("EventsManager is disabled, please enable DEPTHAI_ENABLE_CURL in CMake to use this feature");
}
void EventsManager::setLogResponse(bool logResponse) {
    logger::warn("EventsManager is disabled, please enable DEPTHAI_ENABLE_CURL in CMake to use this feature");
}
}  // namespace utility
}  // namespace dai
#endif
