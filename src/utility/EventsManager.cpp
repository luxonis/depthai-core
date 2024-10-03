#include "depthai/utility/EventsManager.hpp"

#include <chrono>
#include <random>
#include <sstream>

#include "Environment.hpp"
#include "cpr/cpr.h"
#include "depthai/schemas/Event.pb.h"
namespace dai {

namespace utility {
using std::cout;

EventsManager::EventsManager(const std::string& deviceSerialNumber)
    : deviceSerialNumber(deviceSerialNumber), url("https://events-ingest.cloud.luxonis.com/v1/events/"), queueSize(2), sendFrequency(1.0f) {
    sourceAppId = utility::getEnv("AGENT_APP_ID");
    sourceAppIdentifier = utility::getEnv("AGENT_APP_IDENTIFIER");
    token = utility::getEnv("DEPTHAI_HUB_API_KEY");
    eventBufferThread = std::thread([this]() {
        while(true) {
            sendEventBuffer();
            std::this_thread::sleep_for(std::chrono::milliseconds(int(sendFrequency * 1000)));
        }
    });
}

void EventsManager::sendEventBuffer() {
    std::lock_guard<std::mutex> lock(eventBufferMutex);
    if(eventBuffer.empty()) {
        return;
    }
    // Create request
    std::cout << "Sending events" << std::endl;
    cpr::Url url = cpr::Url(this->url);
    cpr::Header header = cpr::Header{{"Content-Type", "application/x-protobuf"}, {"Authorization", "Bearer " + token}};
    cpr::Body body;
    auto batchEvent = std::make_unique<proto::BatchUploadEvents>();
    for(auto& eventM : eventBuffer) {
        auto& event = eventM->event;
        std::cout << "Event: " << event->name() << std::endl;
        std::cout << "Tags: ";
        for(int i = 0; i < event->tags_size(); i++) {
            std::cout << event->tags(i) << " ";
        }
        std::cout << std::endl;
        std::cout << "Extras: ";
        for(int i = 0; i < event->extras_size(); i++) {
            auto extra = event->extras(i).data();
            for(auto& [key, value] : extra) {
                std::cout << key << ": " << value << " ";
            }
        }
        std::cout << std::endl;
        batchEvent->add_events()->Swap(event.get());
    }
    std::string serializedEvent;
    batchEvent->SerializeToString(&serializedEvent);
    body = cpr::Body(serializedEvent);
    cpr::Response r = cpr::Post(url, header, cpr::Body(body));
    if(r.status_code != 200) {
        std::cerr << "Failed to send event: " << r.text << std::endl;
    } else {
        std::cout << "Events sent" << std::endl;
        // upload filek
        auto BatchUploadEventResult = std::make_unique<proto::BatchUploadEventsResult>();
        BatchUploadEventResult->ParseFromString(r.text);
        for(int i = 0; i < BatchUploadEventResult->events_size(); i++) {
            auto eventResult = BatchUploadEventResult->events(i);
            if(eventResult.file_upload_urls().size() > 0) {
                std::cout << "Uploading files" << std::endl;
                for(int j = 0; j < eventResult.file_upload_urls().size(); j++) {
                    cout << "Uploading file: " << eventResult.file_upload_urls(j) << std::endl;
                    cpr::Url fileUrl = cpr::Url(url + eventResult.file_upload_urls(j));

                    // if file struct contains byte data, send it, along with filename and mime type
                    // if it file url, send it directly via url
                    if(eventBuffer[i]->files[j].data.size() > 0) {
                        cpr::Multipart file = cpr::Multipart{{
                            "file",
                            cpr::Buffer{eventBuffer[i]->files[j].data.begin(),
                                        eventBuffer[i]->files[j].data.end(),
                                        eventBuffer[i]->files[j].fileName + eventBuffer[i]->files[j].mimeType},
                        }};
                        cpr::Response r = cpr::Post(fileUrl, header, file);
                        if(r.status_code != 200) {
                            std::cerr << "Failed to upload file: " << r.text << std::endl;
                        }
                    } else {
                        cpr::Response r = cpr::Post(fileUrl, header);
                        if(r.status_code != 200) {
                            std::cerr << "Failed to upload file: " << r.text << std::endl;
                        }
                    }
                }
            }
        }
        eventBuffer.clear();
        std::cout << "Event sent" << std::endl;
    }
}

void EventsManager::sendEvent(const std::string& name,
                              const std::unordered_map<std::string, std::string>& data,
                              const std::vector<std::string>& tags,
                              const std::vector<FileData>& files,
                              const std::shared_ptr<ADatatype>& daiMsg) {
    // Create event
    auto event = std::make_unique<proto::Event>();
    event->set_nonce(createUUID());
    event->set_name(name);
    event->set_created_at(std::chrono::duration_cast<std::chrono::seconds>(std::chrono::system_clock::now().time_since_epoch()).count());
    for(const auto& tag : tags) {
        event->add_tags(tag);
    }
    proto::Extras* extras = event->add_extras();
    auto* extraData = extras->mutable_data();
    for(const auto& [key, value] : data) {
        extraData->insert({key, value});
    }
    if(daiMsg != nullptr) {
        // bytes to stringstream
        std::stringstream ss;
        ss.write((const char*)daiMsg->data->getData().data(), daiMsg->data->getData().size());
        extraData->insert({name, ss.str()});
    }

    event->set_expect_files_num(files.size());

    event->set_source_serial_number(deviceSerialNumber);
    event->set_source_app_id(sourceAppId);
    event->set_source_app_identifier(sourceAppIdentifier);
    // Add event to buffer
    if(eventBuffer.size() <= queueSize) {
        std::lock_guard<std::mutex> lock(eventBufferMutex);
        auto eventMessage = std::make_unique<EventMessage>();
        eventMessage->files = files;
        eventMessage->event = std::move(event);
        eventBuffer.push_back(std::move(eventMessage));
    } else {
        std::cerr << "Event buffer is full, dropping event: " << name << std::endl;
    }
}

void EventsManager::sendSnap(const std::string& name,
                             const std::unordered_map<std::string, std::string>& data,
                             const std::vector<std::string>& tags,
                             const std::vector<FileData>& files,
                             const std::shared_ptr<ADatatype>& daiMsg) {
    std::vector<std::string> tagsTmp = tags;
    tagsTmp.push_back("snap");
    return sendEvent(name, data, tagsTmp, files, daiMsg);
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
}  // namespace utility
}  // namespace dai
