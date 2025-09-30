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
// 3. FileData should be determined, streamlined and wrapped for the final user. Add API

FileData::FileData(const std::string& data, const std::string& fileName, const std::string& mimeType)
    : mimeType(mimeType),
      fileName(fileName),
      data(data),
      size(data.size()),
      checksum(calculateSHA256Checksum(data)),
      classification(proto::event::PrepareFileUploadClass::UNKNOWN_FILE) {}

FileData::FileData(const std::string& filePath, std::string fileName) 
    : fileName(std::move(fileName)) {
    static const std::unordered_map<std::string, std::string> mimeTypeExtensionMap = {
        {".html", "text/html"},
        {".htm",  "text/html"},
        {".css",  "text/css"},
        {".js",   "text/javascript"},
        {".png",  "image/png"},
        {".jpg",  "image/jpeg"},
        {".jpeg", "image/jpeg"},
        {".gif",  "image/gif"},
        {".svg",  "image/svg+xml"},
        {".json", "application/json"},
        {".txt",  "text/plain"}
    };
    // Read the data
    std::ifstream fileStream(filePath, std::ios::binary | std::ios::ate);
    if (!fileStream) {
        return;
    }
    std::streamsize fileSize = fileStream.tellg();
    data.resize(static_cast<size_t>(fileSize));
    fileStream.seekg(0, std::ios::beg);
    fileStream.read(data.data(), fileSize);
    size = data.size();
    checksum = calculateSHA256Checksum(data);
    // Determine the mime type
    auto it = mimeTypeExtensionMap.find(std::filesystem::path(filePath).extension().string());
    if (it != mimeTypeExtensionMap.end()) {
        mimeType = it->second;
    } else {
        mimeType = "application/octet-stream";
    }
    static const std::unordered_set<std::string> imageMimeTypes = {
        "image/jpeg", "image/png", "image/webp", "image/bmp", "image/tiff"
    };
    if (imageMimeTypes.find(mimeType) != imageMimeTypes.end()) {
        classification = proto::event::PrepareFileUploadClass::IMAGE_COLOR;
    } else {
        classification = proto::event::PrepareFileUploadClass::UNKNOWN_FILE;
    }
}

FileData::FileData(const std::shared_ptr<ImgFrame>& imgFrame, std::string fileName)
    : mimeType("image/jpeg"), fileName(std::move(fileName)), classification(proto::event::PrepareFileUploadClass::IMAGE_COLOR) {
    // Convert ImgFrame to bytes
    cv::Mat cvFrame = imgFrame->getCvFrame();
    std::vector<uchar> buffer;
    cv::imencode(".jpg", cvFrame, buffer);

    std::stringstream ss;
    ss.write((const char*)buffer.data(), buffer.size());
    data = ss.str();
    size = data.size();
    checksum = calculateSHA256Checksum(data);
}

FileData::FileData(const std::shared_ptr<EncodedFrame>& encodedFrame, std::string fileName)
    : mimeType("image/jpeg"), fileName(std::move(fileName)), classification(proto::event::PrepareFileUploadClass::IMAGE_COLOR) {
    // Convert EncodedFrame to bytes
    if(encodedFrame->getProfile() != EncodedFrame::Profile::JPEG) {
        logger::error("Only JPEG encoded frames are supported");
        return;
    }
    std::stringstream ss;
    ss.write((const char*)encodedFrame->getData().data(), encodedFrame->getData().size());
    data = ss.str();
    size = data.size();
    checksum = calculateSHA256Checksum(data);
}

FileData::FileData(const std::shared_ptr<NNData>& nnData, std::string fileName)
    : mimeType("application/octet-stream"), fileName(std::move(fileName)), classification(proto::event::PrepareFileUploadClass::UNKNOWN_FILE) {
    // Convert NNData to bytes
    std::stringstream ss;
    ss.write((const char*)nnData->data->getData().data(), nnData->data->getData().size());
    data = ss.str();
    size = data.size();
    checksum = calculateSHA256Checksum(data);
}

FileData::FileData(const std::shared_ptr<ImgDetections>& imgDetections, std::string fileName)
    : mimeType("application/x-protobuf; proto=ImgDetections"), fileName(std::move(fileName)), classification(proto::event::PrepareFileUploadClass::ANNOTATION) {
    // Serialize ImgDetections
    std::vector<uint8_t> imgDetectionsSerialized = imgDetections->serializeProto();
    std::stringstream ss;
    ss.write((const char*)imgDetectionsSerialized.data(), imgDetectionsSerialized.size());
    data = ss.str();
    size = data.size();
    checksum = calculateSHA256Checksum(data);
}

bool FileData::toFile(const std::string& inputPath) {
    if(fileName.empty()) {
        logger::error("Filename is empty");
        return false;
    }
    std::filesystem::path path(inputPath);
    std::string extension = mimeType == "image/jpeg" ? ".jpg" : ".txt";
    // Choose a unique filename
    std::filesystem::path target = path / (fileName + extension);
    for (int i = 1; std::filesystem::exists(target); ++i) {
        logger::warn("File {} exists, trying a new name", target.string());
        target = path / (fileName + "_" + std::to_string(i) + extension);
    }
    std::ofstream fileStream(target, std::ios::binary);
    if (!fileStream) {
        logger::error("Failed to open file for writing: {}", target.string());
        return false;
    }
    fileStream.write(data.data(), static_cast<std::streamsize>(data.size()));
    if (!fileStream) {
        logger::error("Failed to write all data to: {}", target.string());
        return false;
    }
    return true;
}

std::string FileData::calculateSHA256Checksum(const std::string& data) {
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
      verifySsl(true),
      cacheDir("/internal/private"),
      cacheIfCannotSend(false),
      stopUploadThread(false) {
    auto appId = utility::getEnvAs<std::string>("OAKAGENT_APP_ID", "");
    auto containerId = utility::getEnvAs<std::string>("OAKAGENT_CONTAINER_ID", "");
    sourceAppId = appId == "" ? containerId : appId;
    sourceAppIdentifier = utility::getEnvAs<std::string>("OAKAGENT_APP_IDENTIFIER", "");
    token = utility::getEnvAs<std::string>("DEPTHAI_HUB_API_KEY", "");
    dai::Logging::getInstance().logger.set_level(spdlog::level::info);
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
    // TO DO: Determine and add when and how often this fetch should happen !
    logger::info("Fetching configuration limits");
    auto header = cpr::Header();
    header["Authorization"] = "Bearer " + token;
    header["Content-Type"] = "application/x-protobuf";
    cpr::Url requestUrl = static_cast<cpr::Url>(this->url + "/v2/api-usage");
    cpr::Response response = cpr::Get(
        cpr::Url{requestUrl},
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
    if(response.status_code != cpr::status::HTTP_OK) {
        logger::error("Failed to fetch configuration limits, status code: {}", response.status_code);
        return false;
    } else {
        logger::info("Configuration limits fetched successfully");
        auto apiUsage = std::make_unique<proto::event::ApiUsage>();
        apiUsage->ParseFromString(response.text);
        if(logResponse) {
            logger::info("ApiUsage response: \n{}", apiUsage->DebugString());
        }
        // TO DO: Use this data to set the limits
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
    if (response.status_code != cpr::status::HTTP_OK && response.status_code != cpr::status::HTTP_CREATED) {
        logger::error("Failed to prepare a batch of file groups, status code: {}", response.status_code);
    }
    else {
        logger::info("Batch of file groups has been successfully prepared");
        auto fileGroupBatchUpload = std::make_unique<proto::event::BatchFileUploadResult>();
        fileGroupBatchUpload->ParseFromString(response.text);
        if (logResponse) {
            logger::info("BatchFileUploadResult response: \n{}", fileGroupBatchUpload->DebugString());
        }

        // Upload accepted files
        for (int i = 0; i < fileGroupBatchUpload->groups_size(); i++) {
            auto snapData = snapBuffer.at(i);
            auto uploadGroupResult = fileGroupBatchUpload->groups(i);
            // Rejected group
            if (!uploadGroupResult.has_rejected()) {
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
        logger::error("Failed to send event, status code: {}", response.status_code);
    } else {
        logger::info("Event sent successfully");
        if(logResponse) {
            auto eventBatchUploadResults = std::make_unique<proto::event::BatchUploadEventsResult>();
            eventBatchUploadResults->ParseFromString(response.text);
            logger::info("BatchUploadEvents response: \n{}", eventBatchUploadResults->DebugString());
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
    logger::info("Uploading file to: {}", url);
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
        logger::error("Failed to upload file, status code: {}", response.status_code);
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
