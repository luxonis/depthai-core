#include "depthai/utility/EventsManager.hpp"

#include <openssl/sha.h>

#include <chrono>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <utility>

namespace {
std::string generateLocalID(int64_t timestamp) {
    static std::atomic<uint64_t> counter{0};
    std::ostringstream oss;
    oss << timestamp << "_" << std::setw(6) << std::setfill('0') << counter++;
    return oss.str();
}

const std::unordered_map<std::string, std::string> extensionToMimeTypeMap = {
    {".html", "text/html"},
    {".css", "text/css"},
    {".js", "application/javascript"},
    {".png", "image/png"},
    {".jpg", "image/jpeg"},
    {".jpeg", "image/jpeg"},
    {".gif", "image/gif"},
    {".webp", "image/webp"},
    {".bmp", "image/bmp"},
    {".tiff", "image/tiff"},
    {".svg", "image/svg+xml"},
    {".json", "application/json"},
    {".txt", "text/plain"},
    {".annotation", "application/x-protobuf; proto=SnapAnnotation"},
    {"", "application/octet-stream"},
};

const std::unordered_map<std::string, std::string> mimeTypeToExtensionMap = {
    {"text/html", ".html"},
    {"text/css", ".css"},
    {"application/javascript", ".js"},
    {"image/png", ".png"},
    {"image/jpeg", ".jpg"},
    {"image/gif", ".gif"},
    {"image/webp", ".webp"},
    {"image/bmp", ".bmp"},
    {"image/tiff", ".tiff"},
    {"image/svg+xml", ".svg"},
    {"application/json", ".json"},
    {"text/plain", ".txt"},
    {"application/x-protobuf; proto=SnapAnnotation", ".annotation"},
    {"application/octet-stream", ""},
};
}  // namespace

#include "Environment.hpp"
#include "Logging.hpp"
#include "cpr/cpr.h"
#include "depthai/device/DeviceBase.hpp"
#include "depthai/schemas/Event.pb.h"
namespace dai {

namespace utility {

template <typename T, typename... Args>
void addToFileData(std::vector<std::shared_ptr<FileData>>& container, Args&&... args) {
    try {
        container.emplace_back(std::make_shared<T>(std::forward<Args>(args)...));
    } catch(const std::exception& e) {
        logger::error("Failed to create FileData: {}", e.what());
    }
}

void FileGroup::addFile(std::string fileTag, std::string data, std::string mimeType) {
    addToFileData<dai::utility::FileData>(fileData, std::move(data), std::move(fileTag), std::move(mimeType));
}

void FileGroup::addFile(std::string fileTag, std::filesystem::path filePath) {
    addToFileData<dai::utility::FileData>(fileData, std::move(filePath), std::move(fileTag));
}

void FileGroup::addFile(const std::optional<std::string>& fileTag, const std::shared_ptr<ImgFrame>& imgFrame) {
    if(!imgFrame) {
        throw std::invalid_argument("FileGroup::addFile called with null ImgFrame");
    }
    std::string dataFileName = fileTag.value_or("Image");
    addToFileData<dai::utility::FileData>(fileData, imgFrame, std::move(dataFileName));
}

void FileGroup::addFile(const std::optional<std::string>& fileTag, const std::shared_ptr<EncodedFrame>& encodedFrame) {
    if(!encodedFrame) {
        throw std::invalid_argument("FileGroup::addFile called with null EncodedFrame");
    }
    std::string dataFileName = fileTag.value_or("Image");
    addToFileData<dai::utility::FileData>(fileData, encodedFrame, std::move(dataFileName));
}

// void FileGroup::addFile(std::string fileTag, const std::shared_ptr<NNData>& nnData) {
//     addToFileData<dai::utility::FileData>(fileData, nnData, std::move(fileTag));
// }

void FileGroup::addFile(const std::optional<std::string>& fileTag, const std::shared_ptr<ImgDetections>& imgDetections) {
    if(!imgDetections) {
        throw std::invalid_argument("FileGroup::addFile called with null ImgDetections");
    }
    std::string dataFileName = fileTag.value_or("Detections");
    addToFileData<dai::utility::FileData>(fileData, imgDetections, std::move(dataFileName));
}

void FileGroup::addImageDetectionsPair(const std::optional<std::string>& fileTag,
                                       const std::shared_ptr<ImgFrame>& imgFrame,
                                       const std::shared_ptr<ImgDetections>& imgDetections) {
    if(!imgFrame) {
        throw std::invalid_argument("FileGroup::addImageDetectionsPair called with null ImgFrame");
    }
    if(!imgDetections) {
        throw std::invalid_argument("FileGroup::addImageDetectionsPair called with null ImgDetections");
    }
    std::string dataFileName = fileTag.value_or("ImageDetection");
    addToFileData<dai::utility::FileData>(fileData, imgFrame, dataFileName);
    addToFileData<dai::utility::FileData>(fileData, imgDetections, std::move(dataFileName));
}

void FileGroup::addImageDetectionsPair(const std::optional<std::string>& fileTag,
                                       const std::shared_ptr<EncodedFrame>& encodedFrame,
                                       const std::shared_ptr<ImgDetections>& imgDetections) {
    if(!encodedFrame) {
        throw std::invalid_argument("FileGroup::addImageDetectionsPair called with null EncodedFrame");
    }
    if(!imgDetections) {
        throw std::invalid_argument("FileGroup::addImageDetectionsPair called with null ImgDetections");
    }
    std::string dataFileName = fileTag.value_or("ImageDetection");
    addToFileData<dai::utility::FileData>(fileData, encodedFrame, dataFileName);
    addToFileData<dai::utility::FileData>(fileData, imgDetections, std::move(dataFileName));
}

// void FileGroup::addImageNNDataPair(std::string fileTag, const std::shared_ptr<ImgFrame>& imgFrame, const std::shared_ptr<NNData>& nnData) {
//     addToFileData<dai::utility::FileData>(fileData, imgFrame, fileTag);
//     addToFileData<dai::utility::FileData>(fileData, nnData, std::move(fileTag));
// }

// void FileGroup::addImageNNDataPair(std::string fileTag, const std::shared_ptr<EncodedFrame>& encodedFrame, const std::shared_ptr<NNData>& nnData) {
//     addToFileData<dai::utility::FileData>(fileData, encodedFrame, fileTag);
//     addToFileData<dai::utility::FileData>(fileData, nnData, std::move(fileTag));
// }

std::string calculateSHA256Checksum(const std::string& data) {
    unsigned char digest[SHA256_DIGEST_LENGTH];
    SHA256(reinterpret_cast<const unsigned char*>(data.data()), data.size(), digest);

    std::ostringstream oss;
    for(int i = 0; i < SHA256_DIGEST_LENGTH; ++i) {
        oss << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(digest[i]);
    }
    return oss.str();
}

FileData::FileData(std::string data, std::string fileTag, std::string mimeType)
    : mimeType(std::move(mimeType)),
      fileTag(std::move(fileTag)),
      data(std::move(data)),
      size(this->data.size()),
      checksum(calculateSHA256Checksum(this->data)),
      classification(proto::event::PrepareFileUploadClass::UNKNOWN_FILE) {}

FileData::FileData(std::filesystem::path filePath, std::string fileTag) : fileTag(std::move(fileTag)) {
    // Read the data
    std::ifstream fileStream(filePath, std::ios::binary | std::ios::ate);
    if(!fileStream) {
        throw std::runtime_error("File: " + filePath.string() + " doesn't exist");
    }
    std::streamsize fileSize = fileStream.tellg();
    data.resize(static_cast<size_t>(fileSize));
    fileStream.seekg(0, std::ios::beg);
    fileStream.read(data.data(), fileSize);
    size = data.size();
    checksum = calculateSHA256Checksum(data);
    // Determine the mime type
    auto it = extensionToMimeTypeMap.find(filePath.extension().string());
    if(it != extensionToMimeTypeMap.end()) {
        mimeType = it->second;
    } else {
        mimeType = "application/octet-stream";
    }
    static const std::unordered_set<std::string> imageMimeTypes = {"image/jpeg", "image/png", "image/webp", "image/bmp", "image/tiff"};
    if(imageMimeTypes.find(mimeType) != imageMimeTypes.end()) {
        classification = proto::event::PrepareFileUploadClass::IMAGE_COLOR;
    } else if(mimeType == "application/x-protobuf; proto=SnapAnnotation") {
        classification = proto::event::PrepareFileUploadClass::ANNOTATION;
    } else {
        classification = proto::event::PrepareFileUploadClass::UNKNOWN_FILE;
    }
}

FileData::FileData(const std::shared_ptr<ImgFrame>& imgFrame, std::string fileTag)
    : mimeType("image/jpeg"), fileTag(std::move(fileTag)), classification(proto::event::PrepareFileUploadClass::IMAGE_COLOR) {
    // Convert ImgFrame to bytes
    std::vector<uchar> buffer;
    try {
        cv::Mat cvFrame = imgFrame->getCvFrame();
        if(!cv::imencode(".jpg", cvFrame, buffer)) {
            throw std::runtime_error("ImgFrame encoding failed");
        }
    } catch(const cv::Exception& e) {
        throw std::runtime_error(std::string("ImgFrame encoding failed due to OpenCV error: ") + e.what());
    }

    std::stringstream ss;
    ss.write((const char*)buffer.data(), buffer.size());
    data = ss.str();
    size = data.size();
    checksum = calculateSHA256Checksum(data);
}

FileData::FileData(const std::shared_ptr<EncodedFrame>& encodedFrame, std::string fileTag)
    : mimeType("image/jpeg"), fileTag(std::move(fileTag)), classification(proto::event::PrepareFileUploadClass::IMAGE_COLOR) {
    // Convert EncodedFrame to bytes
    if(encodedFrame->getProfile() != EncodedFrame::Profile::JPEG) {
        throw std::runtime_error("Only JPEG encoded frames are supported");
    }
    std::stringstream ss;
    ss.write((const char*)encodedFrame->getData().data(), encodedFrame->getData().size());
    data = ss.str();
    size = data.size();
    checksum = calculateSHA256Checksum(data);
}

// FileData::FileData(const std::shared_ptr<NNData>& nnData, std::string fileTag)
//     : mimeType("application/octet-stream"), fileTag(std::move(fileTag)), classification(proto::event::PrepareFileUploadClass::UNKNOWN_FILE) {
//     // Convert NNData to bytes
//     std::stringstream ss;
//     ss.write((const char*)nnData->data->getData().data(), nnData->data->getData().size());
//     data = ss.str();
//     size = data.size();
//     checksum = calculateSHA256Checksum(data);
// }

FileData::FileData(const std::shared_ptr<ImgDetections>& imgDetections, std::string fileTag)
    : mimeType("application/x-protobuf; proto=SnapAnnotation"), fileTag(std::move(fileTag)), classification(proto::event::PrepareFileUploadClass::ANNOTATION) {
    // Serialize imgDetections object, add it to SnapAnnotation proto
    proto::event::SnapAnnotations snapAnnotation;
    proto::img_detections::ImgDetections imgDetectionsProto;

    if(imgDetections) {
        std::vector<uint8_t> imgDetectionsSerialized = imgDetections->serializeProto();
        if(imgDetectionsProto.ParseFromArray(imgDetectionsSerialized.data(), imgDetectionsSerialized.size())) {
            *snapAnnotation.mutable_detections() = imgDetectionsProto;
        } else {
            throw std::runtime_error("Failed to parse ImgDetections proto from serialized bytes");
        }
    }
    if(!snapAnnotation.SerializeToString(&data)) {
        throw std::runtime_error("Failed to serialize SnapAnnotations proto object to string");
    }
    size = data.size();
    checksum = calculateSHA256Checksum(data);
}

bool FileData::toFile(const std::filesystem::path& inputPath) {
    if(fileTag.empty()) {
        logger::error("FileTag is empty");
        return false;
    }
    std::string extension;
    auto mimeIt = mimeTypeToExtensionMap.find(mimeType);
    if(mimeIt != mimeTypeToExtensionMap.end()) {
        extension = mimeIt->second;
    }
    // Choose a unique fileTag
    std::filesystem::path target = inputPath / (fileTag + extension);
    for(int i = 1; std::filesystem::exists(target); ++i) {
        logger::warn("File {} exists, trying a new name", target.string());
        target = inputPath / (fileTag + "_" + std::to_string(i) + extension);
    }
    std::ofstream fileStream(target, std::ios::binary);
    if(!fileStream) {
        logger::error("Failed to open file for writing: {}", target.string());
        return false;
    }
    fileStream.write(data.data(), static_cast<std::streamsize>(data.size()));
    if(!fileStream) {
        logger::error("Failed to write all data to: {}", target.string());
        return false;
    }
    return true;
}

EventsManager::EventsManager(std::string apiKey, bool uploadCachedOnStart)
    : publishInterval(30.0f),
      logResponse(false),
      verifySsl(true),
      cacheDir("/internal/private"),
      cacheIfCannotSend(false),
      stopUploadThread(false),
      configurationLimitsFetched(false),
      connectionEstablished(false),
      waitingForPendingUploads(false),
      pendingUploadsFinished(false),
      warningStorageBytes(52428800) {
    auto appId = utility::getEnvAs<std::string>("OAKAGENT_APP_ID", "");
    auto containerId = utility::getEnvAs<std::string>("OAKAGENT_CONTAINER_ID", "");
    sourceAppId = appId == "" ? containerId : appId;
    sourceAppIdentifier = utility::getEnvAs<std::string>("OAKAGENT_APP_IDENTIFIER", "");
    // auto connectedDevice = DeviceBase::getFirstAvailableDevice();
    // if(std::get<0>(connectedDevice)) {
    //     sourceSerialNumber = std::get<1>(connectedDevice).getDeviceId();
    // } else {
    //     sourceSerialNumber = "";
    // }
    sourceSerialNumber = "";
    url = utility::getEnvAs<std::string>("DEPTHAI_HUB_EVENTS_BASE_URL", "https://events.cloud.luxonis.com");
    token = apiKey != "" ? apiKey : utility::getEnvAs<std::string>("DEPTHAI_HUB_API_KEY", "");
    // Thread handling preparation and uploads
    uploadThread = std::make_unique<std::thread>([this]() {
        // Fetch configuration limits when starting the new thread
        configurationLimitsFetched = fetchConfigurationLimits(true);
        while(!stopUploadThread && configurationLimitsFetched) {
            connectionEstablished = fetchConfigurationLimits(false);
            if(remainingStorageBytes <= warningStorageBytes) {
                logger::warn("Current remaining storage is running low: {} MB", remainingStorageBytes / (1024 * 1024));
            }
            // Add cached snaps (if any) to the snapBuffer
            if(connectionEstablished) {
                uploadCachedSnaps();
            }
            // Prepare the batch first to reduce contention
            std::deque<std::shared_ptr<SnapData>> snapBatch;
            {
                std::scoped_lock lock(snapBufferMutex, uploadFileBatchFuturesMutex);
                const std::size_t size = std::min<std::size_t>(snapBuffer.size(), maxGroupsPerBatch);
                snapBatch.insert(snapBatch.end(), std::make_move_iterator(snapBuffer.begin()), std::make_move_iterator(snapBuffer.begin() + size));
                snapBuffer.erase(snapBuffer.begin(), snapBuffer.begin() + size);

                if(!snapBatch.empty()) {
                    uploadFileBatchFutures.emplace_back(
                        std::async(std::launch::async, [&, inputSnapBatch = std::move(snapBatch)]() mutable { uploadFileBatch(std::move(inputSnapBatch)); }));
                }
            }

            // Clean up finished futures
            {
                std::lock_guard<std::mutex> lock(uploadFileBatchFuturesMutex);
                for(auto iterator = uploadFileBatchFutures.begin(); iterator != uploadFileBatchFutures.end();) {
                    if(iterator->wait_for(std::chrono::seconds(0)) == std::future_status::ready) {
                        iterator->get();
                        iterator = uploadFileBatchFutures.erase(iterator);
                    } else {
                        ++iterator;
                    }
                }
            }

            uploadEventBatch();
            // Check for any remaining snaps or events in snapBuffer & eventBuffer. Also check for pending file uploads.
            if(waitingForPendingUploads) {
                const bool uploadFinished = checkPendingUploadsFinished();
                {
                    std::lock_guard<std::mutex> pendingLock(pendingUploadsMutex);
                    pendingUploadsFinished.store(uploadFinished);
                }
                if(uploadFinished || !connectionEstablished) {
                    pendingUploadsCondition.notify_all();
                }
            }

            std::unique_lock<std::mutex> lock(stopThreadConditionMutex);
            eventBufferCondition.wait_for(lock, std::chrono::seconds(static_cast<int>(this->publishInterval)), [this]() { return stopUploadThread.load(); });
        }
    });
    // If upload on start is not set, clear the previously cached data when starting
    if(!uploadCachedOnStart) {
        clearCachedData(cacheDir);
    }
}

EventsManager::~EventsManager() {
    stopUploadThread = true;
    eventBufferCondition.notify_all();
    pendingUploadsCondition.notify_all();
    if(uploadThread && uploadThread->joinable()) {
        uploadThread->join();
    }
    // Check for pending snaps or events. Warn the user of potential discarded data, suggest using waitForPendingUploads() to avoid this in the future
    if(!checkPendingUploadsFinished()) {
        logger::warn(
            "EventsManager stopped. Pending snaps were discarded. To avoid data loss, use waitForPendingUploads() before destructing the EventsManager");
    }
}

bool EventsManager::fetchConfigurationLimits(const bool retryOnFail) {
    logger::info("Fetching configuration limits");
    if(token.empty()) {
        logger::warn("Missing API Key, please set DEPTHAI_HUB_API_KEY environment variable or set the key when creating the EventsManager");
        return false;
    }
    auto header = cpr::Header();
    header["Authorization"] = "Bearer " + token;
    cpr::Url requestUrl = static_cast<cpr::Url>(this->url + "/v2/api-usage");
    int retryAttempt = 0;
    while(!stopUploadThread) {
        cpr::Response response = cpr::Get(
            cpr::Url{requestUrl},
            cpr::Header{header},
            cpr::VerifySsl(verifySsl),
            cpr::DebugCallback{[](cpr::DebugCallback::InfoType type, std::string data, intptr_t) {
                if(type == cpr::DebugCallback::InfoType::TEXT) {
                    logger::debug("libcurl debug: TEXT: {}", data);
                } else if(type == cpr::DebugCallback::InfoType::HEADER_IN) {
                    logger::debug("libcurl debug: HEADER_IN: {}", data);
                } else if(type == cpr::DebugCallback::InfoType::HEADER_OUT) {
                    logger::debug("libcurl debug: HEADER_OUT: {}", data);
                }
            }},
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
        if(response.error) {
            logger::error("cpr response error - {}: {}", static_cast<int>(response.error.code), response.error.message);
        }
        if(response.status_code != cpr::status::HTTP_OK) {
            logger::error("Failed to fetch configuration limits, status code: {}", response.status_code);
            // Fetching should be retried indefinetly only on startup
            if(!retryOnFail) {
                return false;
            }

            // Apply exponential backoff
            auto factor = std::pow(uploadRetryPolicy.factor, ++retryAttempt);
            std::chrono::milliseconds duration = std::chrono::milliseconds(uploadRetryPolicy.baseDelay.count() * static_cast<int>(factor));
            logger::info("Retrying to fetch configuration limits, (attempt {} in {} ms)", retryAttempt, duration.count());

            std::unique_lock<std::mutex> lock(stopThreadConditionMutex);
            eventBufferCondition.wait_for(lock, duration, [this]() { return stopUploadThread.load(); });
        } else {
            logger::info("Configuration limits fetched successfully");
            auto apiUsage = std::make_unique<proto::event::ApiUsage>();
            apiUsage->ParseFromString(response.text);
            if(logResponse) {
                logger::info("ApiUsage response: \n{}", apiUsage->DebugString());
            }
            // TO DO: Use this data
            maxFileSizeBytes = apiUsage->files().max_file_size_bytes();           //
            remainingStorageBytes = apiUsage->files().remaining_storage_bytes();  //
            bytesPerHour = apiUsage->files().bytes_per_hour_rate();
            uploadsPerHour = apiUsage->files().uploads_per_hour_rate();
            maxGroupsPerBatch = apiUsage->files().groups_per_allocation();         //
            maxFilesPerGroup = apiUsage->files().files_per_group_in_allocation();  //
            eventsPerHour = apiUsage->events().events_per_hour_rate();
            snapsPerHour = apiUsage->events().snaps_per_hour_rate();
            eventsPerRequest = apiUsage->events().events_per_request();  //

            return true;
        }
    }
    return false;
}

void EventsManager::uploadFileBatch(std::deque<std::shared_ptr<SnapData>> inputSnapBatch) {
    // Prepare files for upload
    auto fileGroupBatchPrepare = std::make_unique<proto::event::BatchPrepareFileUpload>();
    if(inputSnapBatch.empty()) {
        return;
    }
    if(token.empty()) {
        logger::warn("Missing API Key, please set DEPTHAI_HUB_API_KEY environment variable or set the key when creating the EventsManager");
        return;
    }
    // Fill the batch with the groups from inputSnapBatch and their corresponding files
    for(size_t i = 0; i < inputSnapBatch.size(); ++i) {
        auto fileGroup = std::make_unique<proto::event::PrepareFileUploadGroup>();
        for(auto& file : inputSnapBatch.at(i)->fileGroup->fileData) {
            auto addedFile = fileGroup->add_files();
            addedFile->set_checksum(file->checksum);
            addedFile->set_mime_type(file->mimeType);
            addedFile->set_size(file->size);
            addedFile->set_filename(file->fileTag);
            addedFile->set_classification(file->classification);
        }
        fileGroupBatchPrepare->add_groups()->Swap(fileGroup.get());
    }

    int retryAttempt = 0;
    while(!stopUploadThread) {
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
        if(response.status_code != cpr::status::HTTP_OK && response.status_code != cpr::status::HTTP_CREATED) {
            logger::error("Failed to prepare a batch of file groups, status code: {}", response.status_code);
            // Apply exponential backoff
            auto factor = std::pow(uploadRetryPolicy.factor, ++retryAttempt);
            std::chrono::milliseconds duration = std::chrono::milliseconds(uploadRetryPolicy.baseDelay.count() * static_cast<int>(factor));
            logger::info("Retrying to prepare a batch of file groups (attempt {} in {} ms)", retryAttempt, duration.count());

            std::unique_lock<std::mutex> lock(stopThreadConditionMutex);
            eventBufferCondition.wait_for(lock, duration, [this]() { return stopUploadThread.load(); });
            // After retrying a defined number of times, we can determine the connection is not established, cache if enabled
            if(retryAttempt >= uploadRetryPolicy.maxAttempts) {
                for(size_t index = 0; index < inputSnapBatch.size(); ++index) {
                    if(inputSnapBatch.at(index)->eventData->onFailure.has_value() && inputSnapBatch.at(index)->eventData->onFailure.value() != nullptr) {
                        auto event = inputSnapBatch.at(index)->eventData->event;
                        std::string serializedPayload;
                        event->SerializeToString(&serializedPayload);
                        inputSnapBatch.at(index)->eventData->onFailure.value()(SendSnapCallbackResult{event->name(),
                                                                                                      event->created_at(),
                                                                                                      inputSnapBatch.at(index)->eventData->localID,
                                                                                                      std::nullopt,
                                                                                                      serializedPayload,
                                                                                                      SendSnapCallbackStatus::FILE_BATCH_PREPARATION_FAILED});
                    }
                }
                if(cacheIfCannotSend) {
                    cacheSnapData(inputSnapBatch);
                } else {
                    logger::warn("Caching is not enabled, dropping snapBatch");
                }
                return;
            }
        } else {
            logger::info("Batch of file groups has been successfully prepared");
            auto prepareBatchResults = std::make_unique<proto::event::BatchFileUploadResult>();
            prepareBatchResults->ParseFromString(response.text);
            if(logResponse) {
                logger::info("BatchFileUploadResult response: \n{}", prepareBatchResults->DebugString());
            }

            // Upload groups of files
            std::vector<std::future<bool>> groupUploadResults;
            for(int i = 0; i < prepareBatchResults->groups_size(); i++) {
                auto snapData = inputSnapBatch.at(i);
                auto prepareGroupResult = prepareBatchResults->groups(i);
                // Skip rejected groups
                if(prepareGroupResult.has_rejected()) {
                    std::string rejectionReason = dai::proto::event::RejectedFileGroupReason_descriptor()
                                                      ->FindValueByNumber(static_cast<int>(prepareGroupResult.rejected().reason()))
                                                      ->name();
                    logger::error("A group has been rejected because of {}", rejectionReason);
                    if(snapData->eventData->onFailure.has_value() && snapData->eventData->onFailure.value() != nullptr) {
                        auto event = snapData->eventData->event;
                        std::string serializedPayload;
                        event->SerializeToString(&serializedPayload);
                        snapData->eventData->onFailure.value()(SendSnapCallbackResult{event->name(),
                                                                                      event->created_at(),
                                                                                      snapData->eventData->localID,
                                                                                      std::nullopt,
                                                                                      serializedPayload,
                                                                                      SendSnapCallbackStatus::GROUP_CONTAINS_REJECTED_FILES});
                    }
                    continue;
                }
                // Handle groups asynchronously
                groupUploadResults.emplace_back(
                    std::async(std::launch::async, [&, snap = std::move(snapData), group = std::move(prepareGroupResult)]() mutable {
                        return uploadGroup(std::move(snap), std::move(group));
                    }));
            }
            // Wait for all of the reponses, indicating the finish of group uploads
            for(auto& uploadResult : groupUploadResults) {
                if(!uploadResult.valid() || !uploadResult.get()) {
                    logger::info("Failed to upload all of the groups in the given batch");
                    // File upload was unsuccesful, cache if enabled
                    if(cacheIfCannotSend) {
                        cacheSnapData(inputSnapBatch);
                    } else {
                        logger::warn("Caching is not enabled, dropping snapBatch");
                    }
                    return;
                }
            }
            return;
        }
    }
}

bool EventsManager::uploadGroup(std::shared_ptr<SnapData> snapData, dai::proto::event::FileUploadGroupResult prepareGroupResult) {
    std::vector<std::future<bool>> fileUploadResults;
    for(int i = 0; i < prepareGroupResult.files_size(); i++) {
        auto prepareFileResult = prepareGroupResult.files(i);
        if(prepareFileResult.result_case() == proto::event::FileUploadResult::kAccepted) {
            // Add an associate file to the event
            auto associateFile = snapData->eventData->event->add_associate_files();
            associateFile->set_id(prepareFileResult.accepted().id());
            // Upload files asynchronously
            fileUploadResults.emplace_back(std::async(
                std::launch::async,
                [&, fileData = std::move(snapData->fileGroup->fileData.at(i)), uploadUrl = std::move(prepareFileResult.accepted().upload_url())]() mutable {
                    return uploadFile(std::move(fileData), std::move(uploadUrl));
                }));
        } else {
            if(snapData->eventData->onFailure.has_value() && snapData->eventData->onFailure.value() != nullptr) {
                auto event = snapData->eventData->event;
                std::string serializedPayload;
                event->SerializeToString(&serializedPayload);
                snapData->eventData->onFailure.value()(SendSnapCallbackResult{event->name(),
                                                                              event->created_at(),
                                                                              snapData->eventData->localID,
                                                                              std::nullopt,
                                                                              serializedPayload,
                                                                              SendSnapCallbackStatus::GROUP_CONTAINS_REJECTED_FILES});
            }
            return false;
        }
    }
    // Wait for all of the results, indicating the finish of file uploads
    for(auto& uploadResult : fileUploadResults) {
        if(!uploadResult.valid() || !uploadResult.get()) {
            logger::info("Failed to upload all of the files in the given group");
            if(snapData->eventData->onFailure.has_value() && snapData->eventData->onFailure.value() != nullptr) {
                auto event = snapData->eventData->event;
                std::string serializedPayload;
                event->SerializeToString(&serializedPayload);
                snapData->eventData->onFailure.value()(SendSnapCallbackResult{event->name(),
                                                                              event->created_at(),
                                                                              snapData->eventData->localID,
                                                                              std::nullopt,
                                                                              serializedPayload,
                                                                              SendSnapCallbackStatus::FILE_UPLOAD_FAILED});
            }
            return false;
        }
    }
    // Once all of the files are uploaded, the event can be sent
    std::lock_guard<std::mutex> lock(eventBufferMutex);
    eventBuffer.push_back(std::move(snapData->eventData));
    return true;
}

bool EventsManager::uploadFile(std::shared_ptr<FileData> fileData, std::string uploadUrl) {
    logger::info("Uploading file {} to: {}", fileData->fileTag, uploadUrl);
    auto header = cpr::Header();
    header["Content-Type"] = fileData->mimeType;
    for(int i = 0; i < uploadRetryPolicy.maxAttempts && !stopUploadThread; ++i) {
        cpr::Response response = cpr::Put(
            cpr::Url{uploadUrl},
            cpr::Body{fileData->data},
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
            logger::error("Failed to upload file {}, status code: {}", fileData->fileTag, response.status_code);
            if(logResponse) {
                logger::info("Response {}", response.text);
            }
            // Apply exponential backoff
            auto factor = std::pow(uploadRetryPolicy.factor, i + 1);
            std::chrono::milliseconds duration = std::chrono::milliseconds(uploadRetryPolicy.baseDelay.count() * static_cast<int>(factor));
            logger::info("Retrying upload of file {}, (attempt {}/{}) in {} ms", fileData->fileTag, i + 1, uploadRetryPolicy.maxAttempts, duration.count());

            std::unique_lock<std::mutex> lock(stopThreadConditionMutex);
            eventBufferCondition.wait_for(lock, duration, [this]() { return stopUploadThread.load(); });
        } else {
            return true;
        }
    }
    return false;
}

void EventsManager::uploadEventBatch() {
    // Add cached events, if any
    if(connectionEstablished) {
        uploadCachedEvents();
    }

    auto eventBatch = std::make_unique<proto::event::BatchUploadEvents>();
    {
        std::lock_guard<std::mutex> lock(eventBufferMutex);
        if(eventBuffer.empty()) {
            return;
        }
        if(token.empty()) {
            logger::warn("Missing API Key, please set DEPTHAI_HUB_API_KEY environment variable or set the key when creating the EventsManager");
            return;
        }
        for(size_t i = 0; i < eventBuffer.size() && i < eventsPerRequest; ++i) {
            eventBatch->add_events()->CopyFrom(*eventBuffer.at(i)->event.get());
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
        // In case the eventBuffer gets too full (dropped connection), cache the events or drop them
        std::lock_guard<std::mutex> lock(eventBufferMutex);
        if(eventBuffer.size() >= EVENT_BUFFER_MAX_SIZE) {
            // failureCallback
            for(size_t index = 0; index < eventBuffer.size(); ++index) {
                if(!eventBuffer.at(index)->onFailure.has_value() || eventBuffer.at(index)->onFailure.value() == nullptr) {
                    continue;
                }
                auto event = eventBuffer.at(index)->event;
                std::string serializedPayload;
                event->SerializeToString(&serializedPayload);
                eventBuffer.at(index)->onFailure.value()(SendSnapCallbackResult{event->name(),
                                                                                event->created_at(),
                                                                                eventBuffer.at(index)->localID,
                                                                                std::nullopt,
                                                                                serializedPayload,
                                                                                SendSnapCallbackStatus::SEND_EVENT_FAILED});
            }
            if(cacheIfCannotSend) {
                cacheEvents();
            } else {
                logger::warn("EventBuffer is full and caching is not enabled, dropping events");
                eventBuffer.clear();
            }
        }
    } else {
        logger::info("Event sent successfully");
        auto eventBatchUploadResults = std::make_unique<proto::event::BatchUploadEventsResult>();
        eventBatchUploadResults->ParseFromString(response.text);
        if(logResponse) {
            logger::info("BatchUploadEvents response: \n{}", eventBatchUploadResults->DebugString());
        }
        std::lock_guard<std::mutex> lock(eventBufferMutex);
        // successCallback
        for(int index = 0; index < eventBatch->events_size(); ++index) {
            bool hasSuccessCallback = eventBuffer.at(index)->onSuccess.has_value() && eventBuffer.at(index)->onSuccess.value() != nullptr;
            bool hasFailureCallback = eventBuffer.at(index)->onFailure.has_value() && eventBuffer.at(index)->onFailure.value() != nullptr;
            auto event = eventBuffer.at(index)->event;
            std::string serializedPayload;
            event->SerializeToString(&serializedPayload);
            if(eventBatchUploadResults->events(index).result_case() == proto::event::EventResult::kAccepted && hasSuccessCallback) {
                eventBuffer.at(index)->onSuccess.value()(SendSnapCallbackResult{event->name(),
                                                                                event->created_at(),
                                                                                eventBuffer.at(index)->localID,
                                                                                eventBatchUploadResults->events(index).accepted().id(),
                                                                                serializedPayload,
                                                                                SendSnapCallbackStatus::SUCCESS});
            } else if(eventBatchUploadResults->events(index).result_case() == proto::event::EventResult::kRejected && hasFailureCallback) {
                eventBuffer.at(index)->onFailure.value()(SendSnapCallbackResult{event->name(),
                                                                                event->created_at(),
                                                                                eventBuffer.at(index)->localID,
                                                                                std::nullopt,
                                                                                serializedPayload,
                                                                                SendSnapCallbackStatus::EVENT_REJECTED});
            }
        }
        eventBuffer.erase(eventBuffer.begin(), eventBuffer.begin() + eventBatch->events_size());
    }
}

std::optional<std::string> EventsManager::sendEvent(const std::string& name,
                                                    const std::vector<std::string>& tags,
                                                    const std::unordered_map<std::string, std::string>& extras,
                                                    const std::vector<std::string>& associateFiles) {
    // Check if the configuration and limits have already been fetched
    if(!configurationLimitsFetched) {
        logger::error("The configuration and limits have not been successfully fetched, event not sent");
        return std::nullopt;
    }

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
    event->set_source_serial_number(sourceSerialNumber);
    event->set_source_app_id(sourceAppId);
    event->set_source_app_identifier(sourceAppIdentifier);
    for(const auto& file : associateFiles) {
        auto addedFile = event->add_associate_files();
        addedFile->set_id(file);
    }
    if(!validateEvent(*event)) {
        logger::error("Failed to send event, validation failed");
        return std::nullopt;
    }

    // Add event to eventBuffer
    std::lock_guard<std::mutex> lock(eventBufferMutex);
    auto eventData = std::make_unique<EventData>();
    std::string localID = generateLocalID(event->created_at());
    eventData->localID = localID;
    eventData->event = std::move(event);
    eventBuffer.push_back(std::move(eventData));
    return localID;
}

std::optional<std::string> EventsManager::sendSnap(const std::string& name,
                                                   const std::shared_ptr<FileGroup> fileGroup,
                                                   const std::vector<std::string>& tags,
                                                   const std::unordered_map<std::string, std::string>& extras,
                                                   const std::function<void(SendSnapCallbackResult)> successCallback,
                                                   const std::function<void(SendSnapCallbackResult)> failureCallback) {
    // Check if the configuration and limits have already been fetched
    if(!configurationLimitsFetched) {
        logger::error("The configuration and limits have not been successfully fetched, snap not sent");
        return std::nullopt;
    }

    // Prepare snapData
    auto snapData = std::make_unique<SnapData>();
    snapData->fileGroup = fileGroup;
    snapData->eventData = std::make_unique<EventData>();
    snapData->eventData->onSuccess = successCallback;
    snapData->eventData->onFailure = failureCallback;
    // Create an event
    snapData->eventData->event = std::make_unique<proto::event::Event>();
    snapData->eventData->event->set_created_at(std::chrono::duration_cast<std::chrono::seconds>(std::chrono::system_clock::now().time_since_epoch()).count());
    std::string localID = generateLocalID(snapData->eventData->event->created_at());
    snapData->eventData->localID = localID;
    snapData->eventData->event->set_name(name);
    snapData->eventData->event->add_tags("snap");
    for(const auto& tag : tags) {
        snapData->eventData->event->add_tags(tag);
    }
    auto* extrasData = snapData->eventData->event->mutable_extras();
    for(const auto& entry : extras) {
        extrasData->insert({entry.first, entry.second});
    }
    snapData->eventData->event->set_source_serial_number(sourceSerialNumber);
    snapData->eventData->event->set_source_app_id(sourceAppId);
    snapData->eventData->event->set_source_app_identifier(sourceAppIdentifier);
    if(!validateEvent(*snapData->eventData->event)) {
        logger::error("Failed to send snap, validation failed");
        return std::nullopt;
    }
    if(fileGroup->fileData.size() > maxFilesPerGroup) {
        logger::error("Failed to send snap, the number of files in a file group {} exceeds {}", fileGroup->fileData.size(), maxFilesPerGroup);
        return std::nullopt;
    } else if(fileGroup->fileData.empty()) {
        logger::error("Failed to send snap, the file group is empty");
        return std::nullopt;
    }
    for(const auto& file : fileGroup->fileData) {
        if(file->size >= maxFileSizeBytes) {
            logger::error(
                "Failed to send snap, file: {} is bigger than the current maximum file size limit: {} kB. To increase your maximum file size, contact support.",
                file->fileTag,
                maxFileSizeBytes / 1024);
            return std::nullopt;
        }
    }
    // Add the snap to snapBuffer
    std::lock_guard<std::mutex> lock(snapBufferMutex);
    snapBuffer.push_back(std::move(snapData));
    return localID;
}

std::optional<std::string> EventsManager::sendSnap(const std::string& name,
                                                   const std::optional<std::string>& fileTag,
                                                   const std::shared_ptr<ImgFrame> imgFrame,
                                                   const std::optional<std::shared_ptr<ImgDetections>>& imgDetections,
                                                   const std::vector<std::string>& tags,
                                                   const std::unordered_map<std::string, std::string>& extras,
                                                   const std::function<void(SendSnapCallbackResult)> successCallback,
                                                   const std::function<void(SendSnapCallbackResult)> failureCallback) {
    // Create a FileGroup and send a snap containing it
    auto fileGroup = std::make_shared<dai::utility::FileGroup>();
    if(imgDetections.has_value()) {
        fileGroup->addImageDetectionsPair(fileTag, imgFrame, imgDetections.value());
    } else {
        fileGroup->addFile(fileTag, imgFrame);
    }

    return sendSnap(name, fileGroup, tags, extras, successCallback, failureCallback);
}

bool EventsManager::validateEvent(const proto::event::Event& inputEvent) {
    // Name
    const auto& name = inputEvent.name();
    if(name.empty()) {
        logger::error("Invalid event name: empty string");
        return false;
    }
    if(name.length() > EVENT_VALIDATION_NAME_LENGTH) {
        logger::error("Invalid event name: length {} exceeds {}", name.length(), EVENT_VALIDATION_NAME_LENGTH);
        return false;
    }

    // Tags
    if(inputEvent.tags_size() > EVENT_VALIDATION_MAX_TAGS) {
        logger::error("Invalid event tags: number of tags {} exceeds {}", inputEvent.tags_size(), EVENT_VALIDATION_MAX_TAGS);
        return false;
    }
    for(int i = 0; i < inputEvent.tags_size(); ++i) {
        const auto& tag = inputEvent.tags(i);
        if(tag.empty()) {
            logger::error("Invalid event tags: tag[{}] empty string", i);
            return false;
        }
        if(tag.length() > EVENT_VALIDATION_TAG_LENGTH) {
            logger::error("Invalid event tags: tag[{}] length {} exceeds {}", i, tag.length(), EVENT_VALIDATION_TAG_LENGTH);
            return false;
        }
    }

    // Event extras
    if(inputEvent.extras_size() > EVENT_VALIDATION_MAX_EXTRAS) {
        logger::error("Invalid event extras: number of extras {} exceeds {}", inputEvent.extras_size(), EVENT_VALIDATION_MAX_EXTRAS);
        return false;
    }
    int index = 0;
    for(const auto& extra : inputEvent.extras()) {
        const auto& key = extra.first;
        const auto& value = extra.second;
        if(key.empty()) {
            logger::error("Invalid event extras: extra[{}] key empty string", index);
            return false;
        }
        if(key.length() > EVENT_VALIDATION_EXTRA_KEY_LENGTH) {
            logger::error("Invalid event extras: extra[{}] key length {} exceeds {}", index, key.length(), EVENT_VALIDATION_EXTRA_KEY_LENGTH);
            return false;
        }
        if(value.length() > EVENT_VALIDATION_EXTRA_VALUE_LENGTH) {
            logger::error("Invalid event extras: extra[{}] value length {} exceeds {}", index, value.length(), EVENT_VALIDATION_EXTRA_VALUE_LENGTH);
            return false;
        }
        index++;
    }

    // Associate files
    if(inputEvent.associate_files_size() > EVENT_VALIDATION_MAX_ASSOCIATE_FILES) {
        logger::error(
            "Invalid associate files: number of associate files {} exceeds {}", inputEvent.associate_files_size(), EVENT_VALIDATION_MAX_ASSOCIATE_FILES);
        return false;
    }

    return true;
}

bool EventsManager::waitForPendingUploads(uint64_t timeoutMs) {
    {
        std::lock_guard<std::mutex> lock(pendingUploadsMutex);
        pendingUploadsFinished = checkPendingUploadsFinished();
        if(pendingUploadsFinished) {
            return true;
        }
        waitingForPendingUploads = true;
    }

    std::unique_lock<std::mutex> lock(pendingUploadsMutex);
    if(timeoutMs > 0) {
        pendingUploadsCondition.wait_for(lock, std::chrono::milliseconds(timeoutMs), [this]() {
            return pendingUploadsFinished.load() || !connectionEstablished.load() || stopUploadThread.load();
        });
    } else {
        pendingUploadsCondition.wait(lock, [this]() { return pendingUploadsFinished.load() || !connectionEstablished.load() || stopUploadThread.load(); });
    }

    if(!pendingUploadsFinished.load() && !connectionEstablished.load()) {
        logger::warn("waitForPendingUploads() exited because connection was lost before pending uploads were finished");
    }

    waitingForPendingUploads = false;
    return checkPendingUploadsFinished();
}

bool EventsManager::checkPendingUploadsFinished() {
    std::scoped_lock lock(snapBufferMutex, uploadFileBatchFuturesMutex, eventBufferMutex);
    if(!snapBuffer.empty()) {
        return false;
    }
    for(const auto& future : uploadFileBatchFutures) {
        // Check if any of the async jobs handling file uploads are not yet finished
        if(future.wait_for(std::chrono::seconds(0)) != std::future_status::ready) {
            return false;
        }
    }
    if(!eventBuffer.empty()) {
        return false;
    }
    return true;
}

void EventsManager::cacheEvents() {
    // Create a unique directory and save the protobuf message for each event in the eventBuffer
    logger::info("Caching events");
    std::lock_guard<std::mutex> lock(eventBufferMutex);
    for(const auto& eventData : eventBuffer) {
        std::filesystem::path inputPath(cacheDir);
        std::filesystem::path path = inputPath / (fmt::format("event_{}_{}", eventData->event->name(), eventData->event->created_at()));
        for(int i = 1; std::filesystem::exists(path); ++i) {
            path = inputPath / (fmt::format("event_{}_{}_{}", eventData->event->name(), eventData->event->created_at(), i));
        }
        std::string eventDir = path.string();
        logger::info("Caching event to {}", eventDir);
        if(!std::filesystem::exists(cacheDir)) {
            std::filesystem::create_directories(cacheDir);
        }
        std::filesystem::create_directory(eventDir);
        std::ofstream eventFile(path / "event.pb", std::ios::binary);
        eventData->event->SerializeToOstream(&eventFile);
    }
    eventBuffer.clear();
}

void EventsManager::cacheSnapData(std::deque<std::shared_ptr<SnapData>>& inputSnapBatch) {
    // Create a unique directory and save the snapData
    logger::info("Caching snapData");
    for(const auto& snap : inputSnapBatch) {
        std::filesystem::path inputPath(cacheDir);
        std::filesystem::path path = inputPath / ("snap_" + snap->eventData->event->name() + "_" + std::to_string(snap->eventData->event->created_at()));
        for(int i = 1; std::filesystem::exists(path); ++i) {
            path =
                inputPath / ("snap_" + snap->eventData->event->name() + "_" + std::to_string(snap->eventData->event->created_at()) + "_" + std::to_string(i));
        }
        std::string snapDir = path.string();
        logger::info("Caching snap to {}", snapDir);
        if(!std::filesystem::exists(cacheDir)) {
            std::filesystem::create_directories(cacheDir);
        }
        std::filesystem::create_directory(snapDir);
        std::ofstream eventFile(path / "snap.pb", std::ios::binary);
        snap->eventData->event->SerializeToOstream(&eventFile);
        for(auto& file : snap->fileGroup->fileData) {
            file->toFile(path);
        }
    }
}

void EventsManager::uploadCachedEvents() {
    // Iterate over the directories in cacheDir, add events to eventBuffer
    if(!checkForCachedData()) {
        return;
    }
    logger::info("Uploading cached events");

    for(const auto& entry : std::filesystem::directory_iterator(cacheDir)) {
        if(!entry.is_directory()) {
            continue;
        }

        if(entry.path().filename().string().rfind("event", 0) == 0) {
            std::ifstream eventFile(entry.path() / "event.pb", std::ios::binary);
            auto eventData = std::make_unique<EventData>();
            auto event = std::make_shared<proto::event::Event>();
            event->ParseFromIstream(&eventFile);

            // Cached events should be added only if there is enough space in the upcoming request
            std::lock_guard<std::mutex> lock(eventBufferMutex);
            if(eventBuffer.size() >= eventsPerRequest) {
                return;
            }

            eventData->event = event;
            eventBuffer.push_back(std::move(eventData));
            // Clear entries added to the eventBuffer
            clearCachedData(entry.path());
        }
    }
}

void EventsManager::uploadCachedSnaps() {
    // Iterate over the directories in cacheDir, add snaps to snapBuffer
    if(!checkForCachedData()) {
        return;
    }
    logger::info("Uploading cached snaps");

    for(const auto& entry : std::filesystem::directory_iterator(cacheDir)) {
        if(!entry.is_directory()) {
            continue;
        }

        if(entry.path().filename().string().rfind("snap", 0) == 0) {
            std::ifstream snapFile(entry.path() / "snap.pb", std::ios::binary);
            auto snapData = std::make_unique<SnapData>();
            auto eventData = std::make_shared<EventData>();
            auto event = std::make_shared<proto::event::Event>();
            auto fileGroup = std::make_shared<dai::utility::FileGroup>();
            event->ParseFromIstream(&snapFile);
            for(const auto& fileEntry : std::filesystem::directory_iterator(entry.path())) {
                if(fileEntry.is_regular_file() && fileEntry.path() != entry.path() / "snap.pb") {
                    auto fileData = std::make_shared<FileData>(fileEntry.path(), fileEntry.path().stem().string());
                    fileGroup->fileData.push_back(fileData);
                }
            }
            snapData->eventData = eventData;
            snapData->eventData->event = event;
            snapData->fileGroup = fileGroup;

            // Cached snaps should be added only if there is enough space in the upcoming request
            std::lock_guard<std::mutex> lock(snapBufferMutex);
            if(snapBuffer.size() >= maxGroupsPerBatch) {
                return;
            }

            snapBuffer.push_back(std::move(snapData));
            // Clear entries added to the snapBuffer
            clearCachedData(entry.path());
        }
    }
}

bool EventsManager::checkForCachedData() {
    if(!std::filesystem::exists(cacheDir)) {
        return false;
    }
    return std::any_of(
        std::filesystem::directory_iterator(cacheDir), std::filesystem::directory_iterator(), [](const auto& entry) { return entry.is_directory(); });
}

void EventsManager::clearCachedData(const std::filesystem::path& directory) {
    if(!checkForCachedData()) {
        return;
    }
    std::error_code ec;
    std::filesystem::remove_all(directory, ec);
    if(ec) {
        logger::error("Failed to remove cache directory {}: {}", directory.string(), ec.message());
    } else {
        logger::info("Cleared cache directory {}", directory.string());
    }
}

void EventsManager::setCacheDir(const std::string& cacheDir) {
    this->cacheDir = cacheDir;
}

void EventsManager::setLogResponse(bool logResponse) {
    this->logResponse = logResponse;
}

void EventsManager::setVerifySsl(bool verifySsl) {
    this->verifySsl = verifySsl;
}

void EventsManager::setCacheIfCannotSend(bool cacheIfCannotSend) {
    this->cacheIfCannotSend = cacheIfCannotSend;
}

}  // namespace utility
}  // namespace dai
