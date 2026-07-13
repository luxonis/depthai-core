#include "depthai/utility/EventsManager.hpp"

#include <stdexcept>
#include <utility>

#include "depthai/schemas/Event.pb.h"

namespace dai::utility {

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

}  // namespace dai::utility
