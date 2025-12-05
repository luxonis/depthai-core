
#include "depthai/pipeline/datatype/SegmentationMask.hpp"

#include <cstring>
#include <optional>
#include <stdexcept>
#include <vector>

#include "depthai/pipeline/datatype/ImgFrame.hpp"

#ifdef DEPTHAI_ENABLE_PROTOBUF
    #include "utility/ProtoSerialize.hpp"
#endif

namespace dai {

SegmentationMask::~SegmentationMask() = default;
SegmentationMask::SegmentationMask() {
    // Set timestamp to now
    setTimestamp(std::chrono::steady_clock::now());
}

SegmentationMask::SegmentationMask(const std::vector<std::uint8_t>& data, size_t width, size_t height) : SegmentationMask() {
    setTimestamp(std::chrono::steady_clock::now());
    setMask(data, width, height);
}

std::size_t SegmentationMask::getWidth() const {
    return width;
}
std::size_t SegmentationMask::getHeight() const {
    return height;
}

void SegmentationMask::setMask(const std::vector<std::uint8_t>& mask, size_t width, size_t height) {
    if(mask.size() != width * height) {
        throw std::runtime_error("SegmentationMask: data size does not match width*height");
    }
    setData(mask);
    this->width = width;
    this->height = height;
}

void SegmentationMask::setMask(dai::ImgFrame& frame) {
    if(frame.getType() != dai::ImgFrame::Type::GRAY8) {
        throw std::runtime_error("SegmentationMask: ImgFrame type must be GRAY8");
    }
    auto dataSpan = frame.getData();
    std::vector<std::uint8_t> vecMask(dataSpan.begin(), dataSpan.end());
    setData(vecMask);
    this->width = frame.getWidth();
    this->height = frame.getHeight();
    this->transformation = frame.transformation;
    setTimestamp(frame.getTimestamp());
    setTimestampDevice(frame.getTimestampDevice());
    setSequenceNum(frame.getSequenceNum());
}

std::optional<std::vector<std::uint8_t>> SegmentationMask::getMaskData() const {
    const auto& d = data->getData();
    std::vector<std::uint8_t> vecMask(d.begin(), d.end());
    if(vecMask.empty()) {
        return std::nullopt;
    }
    return vecMask;
}

std::optional<dai::ImgFrame> SegmentationMask::getMask() const {
    std::optional<std::vector<std::uint8_t>> maskData = getMaskData();
    if(!maskData) {
        return std::nullopt;
    }

    dai::ImgFrame img;
    img.setWidth(static_cast<unsigned int>(width));
    img.setHeight(static_cast<unsigned int>(height));
    img.setType(dai::ImgFrame::Type::GRAY8);
    img.setSequenceNum(getSequenceNum());
    img.setTimestamp(getTimestamp());
    img.setTimestampDevice(getTimestampDevice());
    if(transformation) {
        img.transformation = *transformation;
    }
    img.setData(*maskData);

    return img;
}

#ifdef DEPTHAI_HAVE_OPENCV_SUPPORT

void SegmentationMask::setCvMask(cv::Mat mask) {
    if(mask.type() != CV_8UC1) {
        throw std::runtime_error("SetCvSegmentationMask: Mask must be of INT8 type, got opencv type " + cv::typeToString(mask.type()) + ".");
    }
    std::vector<std::uint8_t> dataVec;
    if(!mask.isContinuous()) {
        for(int i = 0; i < mask.rows; i++) {
            dataVec.insert(dataVec.end(), mask.ptr(i), mask.ptr(i) + mask.cols * mask.elemSize());
        }
    } else {
        dataVec.insert(dataVec.begin(), mask.datastart, mask.dataend);
    }
    setData(dataVec);
    this->width = mask.cols;
    this->height = mask.rows;
}

std::optional<cv::Mat> SegmentationMask::getCvMask(cv::MatAllocator* allocator) {
    if(data->getData().data() == nullptr) {
        return std::nullopt;
    }
    cv::Size size(static_cast<int>(getWidth()), static_cast<int>(getHeight()));
    int type = CV_8UC1;
    if(size.width <= 0 || size.height <= 0) {
        throw std::runtime_error("Segmentation mask metadata not valid (width or height <= 0).");
    }

    const size_t requiredSize = CV_ELEM_SIZE(type) * static_cast<size_t>(size.area());
    const size_t actualSize = data->getSize();

    if(actualSize != requiredSize) {
        throw std::runtime_error("Segmentation mask data size does not match the expected size, required " + std::to_string(requiredSize) + ", actual "
                                 + std::to_string(actualSize) + ".");
    }

    cv::Mat mask;
    mask = cv::Mat(size, type, data->getData().data());
    CV_Assert(mask.type() == CV_8UC1);

    cv::Mat output;
    if(allocator != nullptr) {
        output.allocator = allocator;
    }
    (mask).copyTo(output);
    return output;
}

std::optional<cv::Mat> SegmentationMask::getCvMaskByIndex(uint8_t index, cv::MatAllocator* allocator) {
    std::optional<cv::Mat> mask = getCvMask(allocator);
    if(!mask.has_value()) {
        return std::nullopt;
    }

    cv::Mat indexedMask;
    cv::compare(*mask, index, indexedMask, cv::CmpTypes::CMP_EQ);
    return indexedMask;
}

#endif

void SegmentationMask::serialize(std::vector<std::uint8_t>& metadata, DatatypeEnum& datatype) const {
    metadata = utility::serialize(*this);
    datatype = this->getDatatype();
}

#ifdef DEPTHAI_ENABLE_PROTOBUF
ProtoSerializable::SchemaPair SegmentationMask::serializeSchema() const {
    return utility::serializeSchema(utility::getProtoMessage(this));
}

std::vector<std::uint8_t> SegmentationMask::serializeProto(bool) const {
    return utility::serializeProto(utility::getProtoMessage(this));
}
#endif

}  // namespace dai
