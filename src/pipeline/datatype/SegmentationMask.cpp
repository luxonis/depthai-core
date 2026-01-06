
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

std::optional<dai::ImgFrame> SegmentationMask::getFrame() const {
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

int32_t SegmentationMask::getArea(uint8_t index) const {
    std::optional<std::vector<std::uint8_t>> maskData = getMaskData();
    if(!maskData) {
        return 0;
    }
    int32_t area = 0;
    for(const auto& val : *maskData) {
        if(val == index) {
            area++;
        }
    }
    return area;
}

std::optional<dai::Point2f> SegmentationMask::getCentroid(uint8_t index) const {
    std::optional<std::vector<std::uint8_t>> maskData = getMaskData();
    if(!maskData) {
        return std::nullopt;
    }

    int32_t area = 0;
    int32_t sumX = 0;
    int32_t sumY = 0;
    for(size_t y = 0; y < height; y++) {
        for(size_t x = 0; x < width; x++) {
            size_t idx = y * width + x;
            if((*maskData)[idx] == index) {
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
    std::optional<std::vector<std::uint8_t>> maskData = getMaskData();
    std::vector<uint8_t> uniqueIndices;
    if(!maskData) {
        return uniqueIndices;
    }

    std::vector<bool> indexPresent(256, false);
    for(const auto& val : *maskData) {
        if(!indexPresent[val] && val != 255) {
            indexPresent[val] = true;
            uniqueIndices.push_back(val);
        }
    }
    std::sort(uniqueIndices.begin(), uniqueIndices.end());
    return uniqueIndices;
}

std::optional<std::vector<std::string>> SegmentationMask::getLabels() const {
    return labels;
}

void SegmentationMask::setLabels(const std::vector<std::string>& labels) {
    this->labels = labels;
}

std::optional<std::vector<std::uint8_t>> SegmentationMask::getMaskByIndex(uint8_t index) const {
    std::optional<std::vector<std::uint8_t>> maskData = getMaskData();
    if(!maskData) {
        return std::nullopt;
    }

    std::vector<std::uint8_t> indexedMask(maskData->size(), 0);
    for(size_t i = 0; i < maskData->size(); i++) {
        if((*maskData)[i] == index) {
            indexedMask[i] = 1;
        }
    }
    return indexedMask;
}

std::optional<std::vector<std::uint8_t>> SegmentationMask::getMaskByLabel(const std::string& label) const {
    if(!labels) {
        return std::nullopt;
    }

    auto it = std::find(labels->begin(), labels->end(), label);
    if(it == labels->end()) {
        return std::nullopt;
    }
    return getMaskByIndex(static_cast<uint8_t>(std::distance(labels->begin(), it)));
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

std::vector<std::vector<dai::Point2f>> SegmentationMask::getContour(uint8_t index) {
    std::optional<cv::Mat> mask = getCvMaskByIndex(index);
    if(!mask.has_value()) {
        return {};
    }
    cv::Mat maskCopy = mask->clone();

    std::vector<std::vector<cv::Point>> contours;
    std::vector<std::vector<dai::Point2f>> result;

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

std::optional<dai::RotatedRect> SegmentationMask::getBoundingBox(uint8_t index, bool useContour) {
    dai::RotatedRect box;
    if(useContour) {  // use contours
        std::optional<cv::Mat> mask = getCvMaskByIndex(index);
        if(!mask.has_value()) {
            return std::nullopt;
        }

        cv::Mat maskCopy = mask->clone();
        std::vector<std::vector<cv::Point>> contours;
        cv::findContours(maskCopy, contours, cv::RetrievalModes::RETR_EXTERNAL, cv::ContourApproximationModes::CHAIN_APPROX_SIMPLE);
        if(contours.empty()) {
            return std::nullopt;
        }

        // Find the largest contour
        size_t largestContourIdx = 0;
        double largestArea = 0.0;
        for(size_t i = 0; i < contours.size(); i++) {
            double area = cv::contourArea(contours[i]);
            if(area > largestArea) {
                largestArea = area;
                largestContourIdx = i;
            }
        }
        cv::RotatedRect cvBox = cv::minAreaRect(contours[largestContourIdx]);
        box = dai::RotatedRect(dai::Point2f(cvBox.center.x / static_cast<float>(width), cvBox.center.y / static_cast<float>(height), true),
                               dai::Size2f(cvBox.size.width / static_cast<float>(width), cvBox.size.height / static_cast<float>(height), true),
                               cvBox.angle);
    } else {
        std::optional<std::vector<std::uint8_t>> maskData = getMaskData();
        if(!maskData) {
            return std::nullopt;
        }

        int minX = static_cast<int>(width);
        int minY = static_cast<int>(height);
        int maxX = -1;
        int maxY = -1;
        for(size_t y = 0; y < height; y++) {
            for(size_t x = 0; x < width; x++) {
                size_t idx = y * width + x;
                if((*maskData)[idx] == index) {
                    minX = std::min(static_cast<int>(x), minX);
                    minY = std::min(static_cast<int>(y), minY);
                    maxX = std::max(static_cast<int>(x), maxX);
                    maxY = std::max(static_cast<int>(y), maxY);
                }
            }
        }
        if(maxX == -1 || maxY == -1) {
            return std::nullopt;
        }
        dai::Point2f center((minX + maxX) / 2.0f / static_cast<float>(width), (minY + maxY) / 2.0f / static_cast<float>(height), true);
        dai::Size2f size((maxX - minX) / static_cast<float>(width), (maxY - minY) / static_cast<float>(height), true);
        box = dai::RotatedRect(center, size, 0.0f);
    }

    return box;
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
