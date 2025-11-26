
#include "depthai/pipeline/datatype/SegmentationMask.hpp"

#include <algorithm>
#include <cstring>
#include <optional>
#include <stdexcept>
#include <vector>

#include "depthai/common/RotatedRect.hpp"
#include "depthai/pipeline/datatype/ImgFrame.hpp"

#ifdef DEPTHAI_HAVE_OPENCV_SUPPORT
    #include "utility/ErrorMacros.hpp"
#endif

#ifdef DEPTHAI_ENABLE_PROTOBUF
    #include "utility/ProtoSerialize.hpp"
#endif

namespace dai {

SegmentationMask::~SegmentationMask() = default;
SegmentationMask::SegmentationMask() = default;

SegmentationMask::SegmentationMask(const std::vector<std::uint8_t>& data, const size_t width, const size_t height) : SegmentationMask() {
    setMask(data, width, height);
}

void SegmentationMask::setSize(size_t width, size_t height) {
    this->width = width;
    this->height = height;
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
void SegmentationMask::setMask(span<const std::uint8_t> mask, size_t width, size_t height) {
    if(mask.size() != width * height) {
        throw std::runtime_error("SegmentationMask: data size does not match width*height");
    }
    data->setSize(mask.size());
    std::memcpy(data->getData().data(), mask.data(), mask.size());
    this->width = width;
    this->height = height;
}

span<std::uint8_t> SegmentationMask::prepareMask(size_t width, size_t height) {
    const size_t size = width * height;
    data->setSize(size);
    this->width = width;
    this->height = height;
    return data->getData();
}

void SegmentationMask::setMask(dai::ImgFrame& frame) {
    if(frame.getType() != dai::ImgFrame::Type::GRAY8) {
        throw std::runtime_error("SegmentationMask: ImgFrame type must be GRAY8");
    }
    const size_t width = frame.getWidth();
    const size_t height = frame.getHeight();
    const size_t stride = frame.getStride();
    if(stride == width) {
        setMask(frame.getData(), width, height);
    } else {  // Need to repack the data
        auto dataSpan = frame.getData();
        const size_t packedSize = static_cast<size_t>(width) * static_cast<size_t>(height);
        const size_t minSourceSize = (static_cast<size_t>(height - 1) * static_cast<size_t>(stride)) + static_cast<size_t>(width);
        if(dataSpan.size() < minSourceSize) {
            throw std::runtime_error("SegmentationMask: ImgFrame data size does not match width/height/stride");
        }
        data->setSize(packedSize);
        auto dst = data->getData();
        for(size_t y = 0; y < height; y++) {
            const size_t srcOffset = y * static_cast<size_t>(stride);
            const size_t dstOffset = y * static_cast<size_t>(width);
            std::memcpy(dst.data() + dstOffset, dataSpan.data() + srcOffset, width);
        }
    }
    this->transformation = frame.transformation;
    setTimestamp(frame.getTimestamp());
    setTimestampDevice(frame.getTimestampDevice());
    setTimestampSystem(frame.getTimestampSystem());
    setSequenceNum(frame.getSequenceNum());
}

std::vector<std::uint8_t> SegmentationMask::getMaskData() const {
    const auto& d = data->getData();
    if(d.empty()) {
        return {};
    }
    return std::vector<std::uint8_t>(d.begin(), d.end());
}

dai::ImgFrame SegmentationMask::getFrame() const {
    dai::ImgFrame img;
    img.setSequenceNum(getSequenceNum());
    img.setTimestamp(getTimestamp());
    img.setTimestampDevice(getTimestampDevice());
    img.setTimestampSystem(getTimestampSystem());
    if(transformation) {
        img.transformation = *transformation;
    }
    std::vector<std::uint8_t> maskData = getMaskData();
    if(maskData.empty()) {
        return img;
    }

    img.setWidth(static_cast<unsigned int>(width));
    img.setHeight(static_cast<unsigned int>(height));
    img.setType(dai::ImgFrame::Type::GRAY8);
    img.setData(maskData);

    return img;
}

std::optional<uint32_t> SegmentationMask::getArea(uint8_t index) const {
    std::vector<std::uint8_t> maskData = getMaskData();
    if(maskData.empty()) {
        return std::nullopt;
    }
    uint32_t area = 0;
    for(const auto& val : maskData) {
        if(val == index) {
            area++;
        }
    }
    if(area == 0) {
        return std::nullopt;
    }

    return area;
}

std::optional<dai::Point2f> SegmentationMask::getCentroid(uint8_t index) const {
    std::vector<std::uint8_t> maskData = getMaskData();
    if(maskData.empty()) {
        return std::nullopt;
    }

    int32_t area = 0;
    int32_t sumX = 0;
    int32_t sumY = 0;
    for(size_t y = 0; y < height; y++) {
        for(size_t x = 0; x < width; x++) {
            size_t idx = y * width + x;
            if(maskData[idx] == index) {
                area++;
                sumX += static_cast<int32_t>(x);
                sumY += static_cast<int32_t>(y);
            }
        }
    }
    if(area == 0) {
        return std::nullopt;
    }
    float cx = static_cast<float>(sumX) / static_cast<float>(area) / static_cast<float>(width);
    float cy = static_cast<float>(sumY) / static_cast<float>(area) / static_cast<float>(height);

    return dai::Point2f(cx, cy, true);
}

std::vector<uint8_t> SegmentationMask::getUniqueIndices() const {
    std::vector<std::uint8_t> maskData = getMaskData();
    std::vector<uint8_t> uniqueIndices;
    if(maskData.empty()) {
        return uniqueIndices;
    }

    std::vector<bool> indexPresent(256, false);
    for(const auto& val : maskData) {
        if(!indexPresent[val] && val != 255) {
            indexPresent[val] = true;
            uniqueIndices.push_back(val);
        }
    }
    std::sort(uniqueIndices.begin(), uniqueIndices.end());
    return uniqueIndices;
}

void SegmentationMask::setLabels(const std::vector<std::string>& labels) {
    this->labels = labels;
}

std::vector<std::string> SegmentationMask::getLabels() const {
    return labels;
}

std::vector<std::uint8_t> SegmentationMask::getMaskByIndex(uint8_t index) const {
    std::vector<std::uint8_t> maskData = getMaskData();
    if(maskData.empty()) {
        return {};
    }

    std::vector<std::uint8_t> indexedMask(maskData.size(), 0);
    for(size_t i = 0; i < maskData.size(); i++) {
        if(maskData[i] == index) {
            indexedMask[i] = 1;
        }
    }
    return indexedMask;
}

std::vector<std::uint8_t> SegmentationMask::getMaskByLabel(const std::string& label) const {
    if(labels.empty()) {
        return {};
    }

    auto it = std::find(labels.begin(), labels.end(), label);
    if(it == labels.end()) {
        return {};
    }
    return getMaskByIndex(static_cast<uint8_t>(std::distance(labels.begin(), it)));
}

bool SegmentationMask::hasValidMask() const {
    return data->getSize() == width * height;
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

cv::Mat SegmentationMask::getCvMask(cv::MatAllocator* allocator) {
    cv::Mat mask;
    if(data->getData().data() == nullptr || data->getSize() == 0) {
        return mask;
    }
    cv::Size size(static_cast<int>(getWidth()), static_cast<int>(getHeight()));
    int type = CV_8UC1;

    const size_t requiredSize = CV_ELEM_SIZE(type) * static_cast<size_t>(size.area());
    const size_t actualSize = data->getSize();

    DAI_CHECK_V(actualSize == requiredSize, "Segmentation mask data size does not match the expected size, required {}, actual {}.", requiredSize, actualSize);

    mask = cv::Mat(size, type, data->getData().data());

    cv::Mat output;
    if(allocator != nullptr) {
        output.allocator = allocator;
    }
    (mask).copyTo(output);
    return output;
}

cv::Mat SegmentationMask::getCvMaskByIndex(uint8_t index, cv::MatAllocator* allocator) {
    cv::Mat mask = getCvMask(allocator);
    if(mask.empty()) {
        return cv::Mat();
    }

    cv::Mat indexedMask;
    cv::compare(mask, index, indexedMask, cv::CmpTypes::CMP_EQ);
    return indexedMask;
}

std::vector<std::vector<dai::Point2f>> SegmentationMask::getContour(uint8_t index) {
    std::vector<std::vector<dai::Point2f>> result;
    cv::Mat mask = getCvMaskByIndex(index);
    if(mask.empty()) {
        return result;
    }
    cv::Mat maskCopy = mask.clone();
    std::vector<std::vector<cv::Point>> contours;

    cv::findContours(maskCopy, contours, cv::RetrievalModes::RETR_EXTERNAL, cv::ContourApproximationModes::CHAIN_APPROX_SIMPLE);
    for(const auto& contour : contours) {
        std::vector<dai::Point2f> daiContour;
        for(const auto& point : contour) {
            daiContour.emplace_back(static_cast<float>(point.x), static_cast<float>(point.y), false);
        }
        result.emplace_back(std::move(daiContour));
    }

    return result;
}

std::vector<dai::RotatedRect> SegmentationMask::getBoundingBoxes(uint8_t index, bool calculateRotation) {
    std::vector<dai::RotatedRect> boxes;
    cv::Mat mask = getCvMaskByIndex(index);
    if(mask.empty()) {
        return {};
    }

    cv::Mat maskCopy = mask.clone();
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(maskCopy, contours, cv::RetrievalModes::RETR_EXTERNAL, cv::ContourApproximationModes::CHAIN_APPROX_SIMPLE);
    if(contours.empty()) {
        return {};
    }
    const float widthF = static_cast<float>(width);
    const float heightF = static_cast<float>(height);

    for(const auto& contour : contours) {
        dai::RotatedRect box;
        if(calculateRotation) {
            cv::RotatedRect cvBox = cv::minAreaRect(contour);
            box = {dai::Point2f(cvBox.center.x / widthF, cvBox.center.y / heightF, true),
                   dai::Size2f(cvBox.size.width / widthF, cvBox.size.height / heightF, true),
                   cvBox.angle};
        } else {
            cv::Rect boundingRect = cv::boundingRect(contour);
            if(boundingRect.width == 0 || boundingRect.height == 0) {
                continue;
            }
            box = {dai::Point2f((boundingRect.x + boundingRect.width / 2.0f) / widthF, (boundingRect.y + boundingRect.height / 2.0f) / heightF, true),
                   dai::Size2f(boundingRect.width / widthF, boundingRect.height / heightF, true),
                   0.0f};
        }

        boxes.push_back(box);
    }

    return boxes;
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
