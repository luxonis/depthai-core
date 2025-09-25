#include "depthai/utility/EventsManager.hpp"

#include <chrono>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <random>
#include <sstream>
#include <utility>
#include <openssl/sha.h>

#include "Environment.hpp"
#include "Logging.hpp"
#include "cpr/cpr.h"
#include "depthai/schemas/Event.pb.h"
namespace dai {

namespace utility {
using std::move;

// TO DO:
// 1. ADD CHECKS FOR STUFF AS VALIDATION RULES IN CLICKUP DOCS 
// 2. Add the newly updated file limits, event limits, api usage)
// 3. FileData should be determined, streamlined and wrapped for the final user 

FileData::FileData(const std::string& data, const std::string& fileName, const std::string& mimeType)
    : mimeType(mimeType), fileName(fileName), data(data) {
    size = data.size();
    classification = proto::event::PrepareFileUploadClass::UNKNOWN_FILE;
    checksum = CalculateSHA256Checksum(data);
}

FileData::FileData(std::string fileUrl) : data(std::move(fileUrl)) {
    fileName = std::filesystem::path(data).filename().string();
    static std::map<std::string, std::string> mimeTypes = {{".html", "text/html"},
                                                           {".htm", "text/html"},
                                                           {".css", "text/css"},
                                                           {".js", "application/javascript"},
                                                           {".png", "image/png"},
                                                           {".jpg", "image/jpeg"},
                                                           {".jpeg", "image/jpeg"},
                                                           {".gif", "image/gif"},
                                                           {".svg", "image/svg+xml"},
                                                           {".json", "application/json"},
                                                           {".txt", "text/plain"}};
    auto ext = std::filesystem::path(data).extension().string();
    auto it = mimeTypes.find(ext);
    mimeType = "application/octet-stream";
    if(it != mimeTypes.end()) {
        mimeType = it->second;
    }
}

FileData::FileData(const std::shared_ptr<ImgFrame>& imgFrame, std::string fileName)
    : mimeType("image/jpeg"), fileName(std::move(fileName)), classification(proto::event::PrepareFileUploadClass::IMAGE_COLOR) {
    // Convert ImgFrame to bytes
    cv::Mat cvFrame = imgFrame->getCvFrame();
    std::vector<uchar> buf;
    cv::imencode(".jpg", cvFrame, buf);

    std::stringstream ss;
    ss.write((const char*)buf.data(), buf.size());
    data = ss.str();
    size = data.size();
    checksum = CalculateSHA256Checksum(data);
}

FileData::FileData(const std::shared_ptr<EncodedFrame>& encodedFrame, std::string fileName)
    : fileName(std::move(fileName)) {//, type(EventDataType::ENCODED_FRAME) {
    // Convert EncodedFrame to bytes
    if(encodedFrame->getProfile() != EncodedFrame::Profile::JPEG) {
        logger::error("Only JPEG encoded frames are supported");
        return;
    }
    std::stringstream ss;
    ss.write((const char*)encodedFrame->getData().data(), encodedFrame->getData().size());
    data = ss.str();
    mimeType = "image/jpeg";
}

FileData::FileData(const std::shared_ptr<NNData>& nnData, std::string fileName)
    : mimeType("application/octet-stream"), fileName(std::move(fileName)) {//, type(EventDataType::NN_DATA) {
    // Convert NNData to bytes
    std::stringstream ss;
    ss.write((const char*)nnData->data->getData().data(), nnData->data->getData().size());
    data = ss.str();
}

bool FileData::toFile(const std::string& path) {
    // check if filename is not empty
    if(fileName.empty()) {
        logger::error("Filename is empty");
        return false;
    }
    std::filesystem::path p(path);
    if(true) {//type == EventDataType::FILE_URL) {
        // get the filename from the url
        std::filesystem::copy(data, p / fileName);
    } else {
        std::string extension = mimeType == "image/jpeg" ? ".jpg" : ".txt";
        // check if file exists, if yes, append a number to the filename
        std::string fileNameTmp = fileName;
        int i = 0;
        while(std::filesystem::exists(p / (fileNameTmp + extension))) {
            logger::warn("File {} already exists, appending number to filename", fileNameTmp);
            fileNameTmp = fileName + "_" + std::to_string(i);
            i++;
        }
        std::ofstream fileStream(p / (fileNameTmp + extension), std::ios::binary);
        fileStream.write(data.data(), data.size());
    }
    return true;
}

std::string FileData::CalculateSHA256Checksum(const std::string& data) {
    unsigned char digest[SHA256_DIGEST_LENGTH];
    SHA256(reinterpret_cast<const unsigned char*>(data.data()), data.size(), digest);

    std::ostringstream oss;
    for (int i = 0; i < SHA256_DIGEST_LENGTH; ++i) {
        oss << std::hex << std::setw(2) << std::setfill('0')
            << static_cast<int>(digest[i]);
    }
    return oss.str();
}


EventsManager::EventsManager(std::string url, bool uploadCachedOnStart, float publishInterval)
    : url(std::move(url)),
      queueSize(10),
      publishInterval(publishInterval),
      logResponse(false),
      logUploadResults(false),
      verifySsl(true),
      cacheDir("/internal/private"),
      cacheIfCannotSend(false),
      stopUploadThread(false) {
    sourceAppId = utility::getEnvAs<std::string>("OAKAGENT_APP_VERSION", "");
    sourceAppIdentifier = utility::getEnvAs<std::string>("OAKAGENT_APP_IDENTIFIER", "");
    token = utility::getEnvAs<std::string>("DEPTHAI_HUB_API_KEY", "");
    std::cout << "LOGGER LEVEL" << logger::get_level() << "\n";
    dai::Logging::getInstance().logger.set_level(spdlog::level::info);
    std::cout << "LOGGER LEVEL" << logger::get_level() << "\n";
    uploadThread = std::make_unique<std::thread>([this]() {
        while(!stopUploadThread) {
            uploadFileBatch();
            uploadEventBatch();
            std::unique_lock<std::mutex> lock(stopThreadConditionMutex);
            eventBufferCondition.wait_for(lock,
                std::chrono::seconds(static_cast<int>(this->publishInterval)),
                [this]() {
                    return stopUploadThread.load();
                });
        }
    });
    if(uploadCachedOnStart) {
        uploadCachedData();
    }
}

EventsManager::~EventsManager() {
    stopUploadThread = true;
    {
        std::unique_lock<std::mutex> lock(stopThreadConditionMutex);
        eventBufferCondition.notify_one();
    }
    if(uploadThread && uploadThread->joinable()) {
        uploadThread->join();
    }
}

bool EventsManager::fetchConfigurationLimits() {
    auto apiUsage = std::make_unique<proto::event::ApiUsage>();
    cpr::Url requestUrl = static_cast<cpr::Url>(this->url + "/v2/api-usage");
    cpr::Response response = cpr::Get(
        cpr::Url{requestUrl},
        //cpr::Body{serializedBatch},
        //cpr::Header{{"Authorization", "Bearer " + token}},
        cpr::VerifySsl(verifySsl),
        cpr::ProgressCallback(
            [&](cpr::cpr_off_t downloadTotal, cpr::cpr_off_t downloadNow, cpr::cpr_off_t uploadTotal, cpr::cpr_off_t uploadNow, intptr_t userdata) -> bool {
                (void)userdata;
                (void)downloadTotal;
                (void)downloadNow;
                (void)uploadTotal;
                (void)uploadNow;
                if(stopUploadThread) {
                    return false;
                }
                return true;
            }));
    if(response.status_code != cpr::status::HTTP_OK) {
        logger::error("Failed to fetch configuration limits: {} {}", response.text, response.status_code);
        return false;
    } else {
        logger::info("Configuration limits fetched successfully");
        if(logResponse) {
            logger::info("Response: {}", response.text);
        }
    }
    return true;
}

void EventsManager::uploadFileBatch() {
    // Prepare files for upload
    auto fileGroupBatchPrepare = std::make_unique<proto::event::BatchPrepareFileUpload>();
    {
        std::lock_guard<std::mutex> lock(snapBufferMutex);
        if (snapBuffer.empty()) {
            return;
        }
        if(token.empty()) {
            logger::warn("Missing token, please set DEPTHAI_HUB_API_KEY environment variable or use setToken method");
            return;
        }
        //if(!checkConnection()) {
            // TO DO: Caching is ignored for now. Fix this later
            //if(cacheIfCannotSend) {
            //    cacheEvents();
            //}
        //    return;
        //}
        // Fill the batch with the groups from snapBuffer and their corresponding files
        for (auto& snapData : snapBuffer) {
            auto fileGroup = std::make_unique<proto::event::PrepareFileUploadGroup>();
            for (auto& file : snapData->fileGroup) {
                auto addedFile = fileGroup->add_files();
                addedFile->set_checksum(file->checksum);
                addedFile->set_mime_type(file->mimeType);
                addedFile->set_size(file->size);
                addedFile->set_filename(file->fileName);
                addedFile->set_classification(file->classification);
            }
            fileGroupBatchPrepare->add_groups()->Swap(fileGroup.get());
        }
    }

    std::string serializedBatch;
    fileGroupBatchPrepare->SerializeToString(&serializedBatch);
    cpr::Url requestUrl = static_cast<cpr::Url>(this->url + "/v2/files/prepare-batch");
    cpr::Response response = cpr::Post(
        cpr::Url{requestUrl},
        cpr::Body{serializedBatch},
        cpr::Header{{"Authorization", "Bearer " + token}},
        cpr::VerifySsl(verifySsl),
        cpr::ProgressCallback(
            [&](cpr::cpr_off_t downloadTotal, cpr::cpr_off_t downloadNow, cpr::cpr_off_t uploadTotal, cpr::cpr_off_t uploadNow, intptr_t userdata) -> bool {
                (void)userdata;
                (void)downloadTotal;
                (void)downloadNow;
                (void)uploadTotal;
                (void)uploadNow;
                if(stopUploadThread) {
                    return false;
                }
                return true;
            }));
    if (response.status_code != cpr::status::HTTP_CREATED) {
        logger::error("Failed to prepare a batch of file groups: {}; status code: {}", response.text, response.status_code);
    }
    else {
        logger::info("Batch of file groups has been successfully prepared");
        if (logResponse) {
            logger::info("Response: {}", response.text);
        }

        // Upload accepted files
        auto fileGroupBatchUpload = std::make_unique<proto::event::BatchFileUploadResult>();
        fileGroupBatchUpload->ParseFromString(response.text);
        for (int i = 0; i < fileGroupBatchUpload->groups_size(); i++) {
            auto snapData = snapBuffer.at(i);
            auto uploadGroupResult = fileGroupBatchUpload->groups(i);
            // Rejected group
            if (uploadGroupResult.has_rejected()) {
                if (!logUploadResults) {
                    continue;
                }
                auto reason = uploadGroupResult.rejected().reason();
                const auto* desc = proto::event::RejectedFileGroupReason_descriptor();
                std::string reasonName = "Unknown RejectedFileGroupReason";
                if (const auto* value = desc->FindValueByNumber(static_cast<int>(reason))) {
                    reasonName = value->name();
                }
                logger::info("File group rejected because of: {}", reasonName);

                for (int j = 0; j < uploadGroupResult.files_size(); j++) {
                    auto uploadFileResult = uploadGroupResult.files(j);
                    if(uploadFileResult.result_case() == proto::event::FileUploadResult::kAccepted) {
                        logger::info("File with id: {} accepted", uploadFileResult.accepted().id());
                    }
                    else if (uploadFileResult.result_case() == proto::event::FileUploadResult::kRejected) {
                        auto reason = uploadFileResult.rejected().reason();
                        const auto* desc = proto::event::RejectedFileReason_descriptor();
                        std::string reasonName = "Unknown RejectedFileReason";
                        if (const auto* value = desc->FindValueByNumber(static_cast<int>(reason))) {
                            reasonName = value->name();
                        }
                        logger::info("File rejected because of: {}; message: {}", reasonName, uploadFileResult.rejected().message());
                    }
                }
            } else {
                for (int j = 0; j < uploadGroupResult.files_size(); j++) {
                    auto uploadFileResult = uploadGroupResult.files(j);
                    if(uploadFileResult.result_case() == proto::event::FileUploadResult::kAccepted) {
                        // TO DO: Make this parallel upload
                        auto addedFile = snapData->event->add_associate_files();
                        addedFile->set_id(uploadFileResult.accepted().id());
                        uploadFile(snapData->fileGroup.at(j), uploadFileResult.accepted().upload_url());
                    }
                }
                if(eventBuffer.size() <= queueSize) {
                    std::lock_guard<std::mutex> lock(eventBufferMutex);
                    eventBuffer.push_back(std::move(snapData->event));
                } else {
                    logger::warn("Event buffer is full, dropping event");
                }
            }
        }

        snapBuffer.clear();
    }
}

void EventsManager::uploadEventBatch() {
    auto eventBatch = std::make_unique<proto::event::BatchUploadEvents>();
    {
        std::lock_guard<std::mutex> lock(eventBufferMutex);
        if(eventBuffer.empty()) {
            return;
        }
        if(token.empty()) {
            logger::warn("Missing token, please set DEPTHAI_HUB_API_KEY environment variable or use setToken method");
            return;
        }
        //if(!checkConnection()) {
            // TO DO: Caching is ignored for now. Fix this later
            //if(cacheIfCannotSend) {
            //    cacheEvents();
            //}
        //    return;
        //}
        for(auto& event : eventBuffer) {
            eventBatch->add_events()->Swap(event.get());
        }
    }
    std::string serializedBatch;
    eventBatch->SerializeToString(&serializedBatch);
    cpr::Url requestUrl = static_cast<cpr::Url>(this->url + "/v2/events");
    cpr::Response response = cpr::Post(
        cpr::Url{requestUrl},
        cpr::Body{serializedBatch},
        cpr::Header{{"Authorization", "Bearer " + token}},
        cpr::VerifySsl(verifySsl),
        cpr::ProgressCallback(
            [&](cpr::cpr_off_t downloadTotal, cpr::cpr_off_t downloadNow, cpr::cpr_off_t uploadTotal, cpr::cpr_off_t uploadNow, intptr_t userdata) -> bool {
                (void)userdata;
                (void)downloadTotal;
                (void)downloadNow;
                (void)uploadTotal;
                (void)uploadNow;
                if(stopUploadThread) {
                    return false;
                }
                return true;
            }));
    if(response.status_code != cpr::status::HTTP_OK) {
        logger::error("Failed to send event: {} {}", response.text, response.status_code);
    } else {
        logger::info("Event sent successfully");
        if(logResponse) {
            logger::info("Response: {}", response.text);
        }

        if (logUploadResults) {
            auto eventBatchUploadResults = std::make_unique<proto::event::BatchUploadEventsResult>();
            eventBatchUploadResults->ParseFromString(response.text);
            for(int i = 0; i < eventBatchUploadResults->events_size(); i++) {
                auto eventUploadResult = eventBatchUploadResults->events(i);
                if(eventUploadResult.result_case() == proto::event::EventResult::kAccepted) {
                    logger::info("Event with id: {} accepted", eventUploadResult.accepted().id());
                }
                else if (eventUploadResult.result_case() == proto::event::EventResult::kRejected) {
                    auto reason = eventUploadResult.rejected().reason();
                    const auto* desc = proto::event::RejectedEventReason_descriptor();
                    std::string reasonName = "Unknown RejectedEventReason";
                    if (const auto* value = desc->FindValueByNumber(static_cast<int>(reason))) {
                        reasonName = value->name();
                    }
                    logger::info("Event rejected because of: {}; message: {}", reasonName, eventUploadResult.rejected().message());
                }
            }
        }

        // TO DO: Caching is ignored for now. Fix this later
        //for(auto& eventM : eventBuffer) {
        //    if(!eventM->cachePath.empty() && std::filesystem::exists(eventM->cachePath)) {
        //        std::filesystem::remove_all(eventM->cachePath);
        //    }
        //}

        eventBuffer.clear();
    }
}

bool EventsManager::sendEvent(const std::string& name,
                              const std::vector<std::string>& tags,
                              const std::unordered_map<std::string, std::string>& extras,
                              const std::string& deviceSerialNo,
                              const std::vector<std::string>& associateFiles) {
    // Create an event
    auto event = std::make_unique<proto::event::Event>();
    event->set_created_at(std::chrono::duration_cast<std::chrono::seconds>(std::chrono::system_clock::now().time_since_epoch()).count());
    event->set_name(name);
    for(const auto& tag : tags) {
        event->add_tags(tag);
    }
    auto* extrasData = event->mutable_extras();
    for(const auto& entry : extras) {
        extrasData->insert({entry.first, entry.second});
    }
    event->set_source_serial_number(deviceSerialNo.empty() ? deviceSerialNumber : deviceSerialNo);
    event->set_source_app_id(sourceAppId);
    event->set_source_app_identifier(sourceAppIdentifier);
    for (const auto& file : associateFiles) {
        auto addedFile = event->add_associate_files();
        addedFile->set_id(file);
    }
    // Add event to eventBuffer
    if(eventBuffer.size() <= queueSize) {
        std::lock_guard<std::mutex> lock(eventBufferMutex);
        eventBuffer.push_back(std::move(event));
    } else {
        logger::warn("Event buffer is full, dropping event");
        return false;
    }
    return true;
}

bool EventsManager::sendSnap(const std::string& name,
                             const std::vector<std::string>& tags,
                             const std::unordered_map<std::string, std::string>& extras,
                             const std::string& deviceSerialNo,
                             const std::vector<std::shared_ptr<FileData>>& fileGroup) {
    // Prepare snapData
    auto snapData = std::make_unique<SnapData>();
    snapData->fileGroup = fileGroup;
    // Create an event
    snapData->event = std::make_unique<proto::event::Event>();
    snapData->event->set_created_at(std::chrono::duration_cast<std::chrono::seconds>(std::chrono::system_clock::now().time_since_epoch()).count());
    snapData->event->set_name(name);
    snapData->event->add_tags("snap");
    for(const auto& tag : tags) {
        snapData->event->add_tags(tag);
    }
    auto* extrasData = snapData->event->mutable_extras();
    for(const auto& entry : extras) {
        extrasData->insert({entry.first, entry.second});
    }
    snapData->event->set_source_serial_number(deviceSerialNo.empty() ? deviceSerialNumber : deviceSerialNo);
    snapData->event->set_source_app_id(sourceAppId);
    snapData->event->set_source_app_identifier(sourceAppIdentifier);
    // Add the snap to snapBuffer
    // TO DO: Should a snap buffer be limited by size like eventbuffer
    if(snapBuffer.size() <= queueSize) {
        std::lock_guard<std::mutex> lock(snapBufferMutex);
        snapBuffer.push_back(std::move(snapData));
    } else {
        logger::warn("Snap buffer is full, dropping snap");
        return false;
    }
    return true;
}

void EventsManager::uploadFile(const std::shared_ptr<FileData>& file, const std::string& url) {
    logger::info("Uploading file: to {}", url);
    auto header = cpr::Header();
    header["File-Size"] = file->size;
    header["Content-Type"] = file->mimeType;
    cpr::Response response = cpr::Put(
        cpr::Url{url},
        cpr::Body{file->data},
        cpr::Header{header},
        cpr::VerifySsl(verifySsl),
        cpr::ProgressCallback(
            [&](cpr::cpr_off_t downloadTotal, cpr::cpr_off_t downloadNow, cpr::cpr_off_t uploadTotal, cpr::cpr_off_t uploadNow, intptr_t userdata) -> bool {
                (void)userdata;
                (void)downloadTotal;
                (void)downloadNow;
                (void)uploadTotal;
                (void)uploadNow;
                if(stopUploadThread) {
                    return false;
                }
                return true;
            }));
    if(response.status_code != cpr::status::HTTP_OK && response.status_code != cpr::status::HTTP_CREATED) {
        logger::error("Failed to upload file: {} ; error code: {}", response.text, response.status_code);
    } else {
        if(logResponse) {
            logger::info("Response: {}", response.text);
        }
    }
}

void EventsManager::cacheEvents() {
    /*
    logger::info("Caching events");
    // for each event, create a unique directory, save protobuf message and associated files
    // TO DO: Make this using associate file field of proto::event::Event
    std::lock_guard<std::mutex> lock(uploadMutex);
    for(auto& eventM : eventBuffer) {
        auto& event = eventM->event;
        auto& data = eventM->data;
        std::filesystem::path p(cacheDir);
        p = p / ("event_" + event->name() + "_" + event->nonce());
        std::string eventDir = p.string();
        logger::info("Caching event to {}", eventDir);
        if(!std::filesystem::exists(cacheDir)) {
            std::filesystem::create_directories(cacheDir);
        }
        std::filesystem::create_directory(eventDir);
        std::ofstream eventFile(p / "event.pb", std::ios::binary);
        event->SerializeToOstream(&eventFile);
        for(auto& file : data) {
            file->toFile(eventDir);
        }
    }
    eventBuffer.clear();
    */
}

void EventsManager::uploadCachedData() {
    /*
    // iterate over all directories in cacheDir, read event.pb and associated files, and send them
    logger::info("Uploading cached data");
    if(!checkConnection()) {
        return;
    }
    // check if cacheDir exists
    if(!std::filesystem::exists(cacheDir)) {
        logger::warn("Cache directory does not exist");
        return;
    }
    for(const auto& entry : std::filesystem::directory_iterator(cacheDir)) {
        if(entry.is_directory()) {
            const auto& eventDir = entry.path();
            std::ifstream eventFile(eventDir / "event.pb", std::ios::binary);
            proto::event::Event event;
            event.ParseFromIstream(&eventFile);
            std::vector<std::shared_ptr<EventData>> data;
            for(const auto& fileEntry : std::filesystem::directory_iterator(eventDir)) {
                if(fileEntry.is_regular_file() && fileEntry.path() != eventDir / "event.pb") {
                    auto fileData = std::make_shared<EventData>(fileEntry.path().string());
                    data.push_back(fileData);
                }
            }
            std::lock_guard<std::mutex> lock(eventBufferMutex);
            auto eventPtr = std::make_shared<proto::event::Event>(event);
            auto eventMessage = std::make_shared<EventMessage>();
            eventMessage->event = eventPtr;
            eventMessage->data = data;
            eventMessage->cachePath = eventDir.string();
            eventBuffer.push_back(eventMessage);
        }
    }
    */
}

bool EventsManager::checkForCachedData() {
    if(!std::filesystem::exists(cacheDir)) {
        logger::warn("Cache directory does not exist");
        return false;
    }
    return std::any_of(
        std::filesystem::directory_iterator(cacheDir), std::filesystem::directory_iterator(), [](const auto& entry) { return entry.is_directory(); });
}

void EventsManager::setCacheDir(const std::string& cacheDir) {
    this->cacheDir = cacheDir;
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

void EventsManager::setQueueSize(uint64_t queueSize) {
    this->queueSize = queueSize;
}

void EventsManager::setLogResponse(bool logResponse) {
    this->logResponse = logResponse;
}

void EventsManager::setLogUploadResults(bool logUploadResults) {
    this->logUploadResults = logUploadResults;
}

void EventsManager::setDeviceSerialNumber(const std::string& deviceSerialNumber) {
    this->deviceSerialNumber = deviceSerialNumber;
}

void EventsManager::setVerifySsl(bool verifySsl) {
    this->verifySsl = verifySsl;
}

void EventsManager::setCacheIfCannotSend(bool cacheIfCannotSend) {
    this->cacheIfCannotSend = cacheIfCannotSend;
}

}  // namespace utility
}  // namespace dai
