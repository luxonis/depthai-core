#include "depthai/utility/EventsManager.hpp"

#include <stdexcept>
#include <utility>

#include "depthai/schemas/Event.pb.h"
#include "utility/Logging.hpp"

namespace dai::utility {

template <typename... Args>
void addToFileData(std::vector<std::shared_ptr<FileData>>& container, Args&&... args) {
    try {
        container.emplace_back(std::make_shared<FileData>(std::forward<Args>(args)...));
    } catch(const std::exception& e) {
        logger::error("Failed to create FileData: {}", e.what());
    }
}

FileData::FileData(const std::shared_ptr<ImgFrame>& imgFrame, std::string fileTag)
    : mimeType("image/jpeg"), fileTag(std::move(fileTag)), classification(proto::event::PrepareFileUploadClass::IMAGE_COLOR) {
    std::vector<uchar> buffer;
    try {
        cv::Mat cvFrame = imgFrame->getCvFrame();
        if(!cv::imencode(".jpg", cvFrame, buffer)) {
            throw std::runtime_error("ImgFrame encoding failed");
        }
    } catch(const cv::Exception& e) {
        throw std::runtime_error(std::string("ImgFrame encoding failed due to OpenCV error: ") + e.what());
    }

    data.assign(reinterpret_cast<const char*>(buffer.data()), buffer.size());
    size = data.size();
    checksum = calculateSHA256Checksum(data);
}

void FileGroup::addFile(const std::optional<std::string>& fileTag, const std::shared_ptr<ImgFrame>& imgFrame) {
    if(!imgFrame) {
        throw std::invalid_argument("FileGroup::addFile called with null ImgFrame");
    }
    addToFileData(fileData, imgFrame, fileTag.value_or("Image"));
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
    addToFileData(fileData, imgFrame, dataFileName);
    addToFileData(fileData, imgDetections, std::move(dataFileName));
}

std::optional<std::string> EventsManager::sendSnap(const std::string& name,
                                                   const std::optional<std::string>& fileTag,
                                                   const std::shared_ptr<ImgFrame> imgFrame,
                                                   const std::optional<std::shared_ptr<ImgDetections>>& imgDetections,
                                                   const std::vector<std::string>& tags,
                                                   const std::unordered_map<std::string, std::string>& extras,
                                                   const std::function<void(SendSnapCallbackResult)> successCallback,
                                                   const std::function<void(SendSnapCallbackResult)> failureCallback) {
    auto fileGroup = std::make_shared<FileGroup>();
    if(imgDetections.has_value()) {
        fileGroup->addImageDetectionsPair(fileTag, imgFrame, imgDetections.value());
    } else {
        fileGroup->addFile(fileTag, imgFrame);
    }

    return sendSnap(name, fileGroup, tags, extras, successCallback, failureCallback);
}

}  // namespace dai::utility
