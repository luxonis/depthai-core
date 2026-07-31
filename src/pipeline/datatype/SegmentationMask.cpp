
#include "depthai/pipeline/datatype/SegmentationMask.hpp"

#include <algorithm>
#include <cstring>
#include <memory>
#include <optional>
#include <stdexcept>
#include <utility>
#include <vector>

#include "depthai/common/RotatedRect.hpp"
#include "depthai/pipeline/datatype/ImgFrame.hpp"

#ifdef DEPTHAI_ENABLE_PROTOBUF
    #include "utility/ProtoSerialize.hpp"
#endif

namespace dai {

SegmentationMask::~SegmentationMask() = default;
SegmentationMask::SegmentationMask() = default;

SegmentationMask::SegmentationMask(const std::vector<std::uint8_t>& data, const size_t width, const size_t height) : SegmentationMask() {
    setMask(data, width, height);
}

SegmentationMask SegmentationMask::transformTo(const ImgTransformation& target) const {
    return TransformableCRTP<SegmentationMask>::transformTo(target);
}

void SegmentationMask::setSize(size_t width, size_t height) {
    this->width = width;
    this->height = height;
}

void SegmentationMask::transformToInternal(const ImgTransformation&) {
    // Transform the segmentation mask by using ImageAlign for faster processing.
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
    auto vecMask = mask;          // Avoid mutating shared storage by moving a copy of the input into a new buffer
    setData(std::move(vecMask));  // Call the rvalue overload to allocate a new memory holder
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
    this->setBufferMetadataFrom(&frame);
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
    img.setBufferMetadataFrom(this);
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
