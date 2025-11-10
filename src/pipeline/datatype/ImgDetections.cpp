#include "depthai/pipeline/datatype/ImgDetections.hpp"

#include <vector>

#include "depthai/common/KeypointsList.hpp"
#include "depthai/common/Point2f.hpp"
#include "depthai/common/Rect.hpp"
#include "depthai/common/RotatedRect.hpp"
#include "depthai/common/Size2f.hpp"
#include "depthai/pipeline/datatype/ImgAnnotations.hpp"
#include "depthai/pipeline/datatype/ImgFrame.hpp"

#ifdef DEPTHAI_ENABLE_PROTOBUF
    #include "depthai/schemas/ImgDetections.pb.h"
    #include "utility/ProtoSerialize.hpp"
#endif

namespace dai {

// ImgDetection functions

ImgDetection::ImgDetection(const dai::RotatedRect& boundingBox, float conf = 0.f, uint32_t label = 0) {
    setBoundingBox(boundingBox);
    this->confidence = conf;
    this->label = label;
}
ImgDetection::ImgDetection(const dai::RotatedRect& boundingBox, std::string labelName, float conf = 0.f, uint32_t label = 0)
    : ImgDetection(boundingBox, conf, label) {
    this->labelName = std::move(labelName);
}

ImgDetection::ImgDetection(const dai::RotatedRect& boundingBox, const dai::KeypointsList& keypoints, float conf = 0.f, uint32_t label = 0)
    : ImgDetection(boundingBox, conf, label) {
    this->keypoints = keypoints;
}

ImgDetection::ImgDetection(
    const dai::RotatedRect& boundingBox, const dai::KeypointsList& keypoints, std::string labelName, float conf = 0.f, uint32_t label = 0)
    : ImgDetection(boundingBox, conf, label) {
    this->keypoints = keypoints;
    this->labelName = std::move(labelName);
}

void ImgDetection::setBoundingBox(const RotatedRect boundingBox) {
    this->boundingBox = boundingBox;

    std::array<float, 4> outerPoints = this->boundingBox->getOuterRect();
    xmin = outerPoints[0];
    ymin = outerPoints[1];
    xmax = outerPoints[2];
    ymax = outerPoints[3];
}

RotatedRect ImgDetection::getBoundingBox() const {
    if(boundingBox.has_value()) {
        return boundingBox.value();
    } else if(xmin == 0.f && xmax == 0.f && ymin == 0.f && ymax == 0.f) {
        throw std::runtime_error("All bounding box values are zero, no bounding box can be built.");
    }

    // construct from legacy values
    RotatedRect rect = RotatedRect{dai::Rect{Point2f{xmin, ymin}, Point2f{xmax, ymax}}};
    return rect;
}

void ImgDetection::setOuterBoundingBox(const float xmin, const float ymin, const float xmax, const float ymax) {
    this->xmin = xmin;
    this->ymin = ymin;
    this->xmax = xmax;
    this->ymax = ymax;

    Point2f topLeft{xmin, ymin};
    Point2f bottomRight{xmax, ymax};

    this->boundingBox = RotatedRect{Rect{topLeft, bottomRight}};
}

void ImgDetection::setKeypoints(const KeypointsList kp) {
    this->keypoints = kp;
}

void ImgDetection::setKeypoints(const std::vector<Keypoint> kp) {
    this->keypoints = KeypointsList(kp);
}

void ImgDetection::setKeypoints(const std::vector<Keypoint> kps, const std::vector<std::array<uint32_t, 2>> edges) {
    this->keypoints = KeypointsList(kps, edges);
}

void ImgDetection::setKeypoints(const std::vector<Point3f> kps3) {
    this->keypoints = KeypointsList();
    this->keypoints->setKeypoints(kps3);
}

void ImgDetection::setKeypoints(const std::vector<Point2f> kps2) {
    this->keypoints = KeypointsList();
    this->keypoints->setKeypoints(kps2);
}

std::vector<Keypoint> ImgDetection::getKeypoints() const {
    if(keypoints.has_value()) {
        return keypoints->getKeypoints();
    } else {
        return {};
    }
}

void ImgDetection::setEdges(const std::vector<Edge> edges) {
    if(!keypoints.has_value()) {
        throw std::runtime_error("No keypoints set, cannot set edges.");
    }
    keypoints->setEdges(edges);
}

std::vector<Edge> ImgDetection::getEdges() const {
    if(keypoints.has_value()) {
        return keypoints->getEdges();
    } else {
        return {};
    }
}

float ImgDetection::getCenterX() const noexcept {
    return getBoundingBox().center.x;
}

float ImgDetection::getCenterY() const noexcept {
    return getBoundingBox().center.y;
}

float ImgDetection::getWidth() const noexcept {
    return getBoundingBox().size.width;
}

float ImgDetection::getHeight() const noexcept {
    return getBoundingBox().size.height;
}

float ImgDetection::getAngle() const noexcept {
    return getBoundingBox().angle;
}

// ImgDetections functions

size_t ImgDetections::getSegmentationMaskWidth() const {
    return segmentationMaskWidth;
}

size_t ImgDetections::getSegmentationMaskHeight() const {
    return segmentationMaskHeight;
}

void ImgDetections::setMask(const std::vector<std::uint8_t>& mask, size_t width, size_t height) {
    if(mask.size() != width * height) {
        throw std::runtime_error("SegmentationMask: data size does not match width*height");
    }
    setData(mask);
    this->segmentationMaskWidth = width;
    this->segmentationMaskHeight = height;
}

std::optional<std::vector<std::uint8_t>> ImgDetections::getMaskData() const {
    const auto& d = data->getData();
    std::vector<std::uint8_t> vecMask(d.begin(), d.end());
    if(vecMask.empty()) {
        return std::nullopt;
    }
    return vecMask;
}

std::optional<dai::ImgFrame> ImgDetections::getSegmentationMaskAsImgFrame() const {
    std::optional<std::vector<std::uint8_t>> maskData = getMaskData();
    if(!maskData) {
        return std::nullopt;
    }
    dai::ImgFrame img;
    img.setWidth(segmentationMaskWidth);
    img.setHeight(segmentationMaskHeight);
    img.setType(dai::ImgFrame::Type::GRAY8);
    img.setSequenceNum(sequenceNum);
    img.setTimestamp(getTimestamp());
    img.setTimestampDevice(getTimestampDevice());
    img.setData(*maskData);

    return img;
}

// Optional - xtensor support
#ifdef DEPTHAI_XTENSOR_SUPPORT
using XArray2D = xt::xtensor<std::uint8_t, 2, xt::layout_type::row_major>;

std::optional<XArray2D> ImgDetections::getTensorSegmentationMask() const {
    std::optional<std::vector<std::uint8_t>> maskData = getMaskData();
    if(!maskData) {
        return std::nullopt;
    }

    size_t dataSize = (*maskData).size();
    if(dataSize != segmentationMaskWidth * segmentationMaskHeight) {
        throw std::runtime_error("SegmentationMask: data size does not match width*height");
    }

    std::array<std::size_t, 2> shape{segmentationMaskHeight, segmentationMaskWidth};
    auto result = XArray2D::from_shape(shape);
    std::copy(maskData->cbegin(), maskData->cend(), result.begin());
    return result;
}

ImgDetections& ImgDetections::setTensorSegmentationMask(XArray2D mask) {
    data->setSize(mask.size());
    std::vector<uint8_t> dataVec(mask.begin(), mask.end());
    setData(dataVec);
    this->segmentationMaskWidth = mask.shape()[1];
    this->segmentationMaskHeight = mask.shape()[0];
    return *this;
}

std::optional<XArray2D> ImgDetections::getTensorSegmentationMaskByIndex(std::uint8_t index) const {
    std::optional<std::vector<std::uint8_t>> maskData = getMaskData();
    if(!maskData) {
        return std::nullopt;
    }

    if((*maskData).size() != segmentationMaskWidth * segmentationMaskHeight) {
        throw std::runtime_error("SegmentationMask: data size does not match width*height");
    }
    std::array<std::size_t, 2> shape{segmentationMaskHeight, segmentationMaskWidth};
    auto result = XArray2D::from_shape(shape);
    auto dstIt = result.begin();
    auto srcIt = maskData->cbegin();
    for(; dstIt != result.end(); ++dstIt, ++srcIt) {
        *dstIt = static_cast<std::uint8_t>(*srcIt == index);
    }

    return result;
}

#endif

ImgDetections::~ImgDetections() = default;

void ImgDetections::serialize(std::vector<std::uint8_t>& metadata, DatatypeEnum& datatype) const {
    metadata = utility::serialize(*this);
    datatype = DatatypeEnum::ImgDetections;
}

#ifdef DEPTHAI_ENABLE_PROTOBUF
ProtoSerializable::SchemaPair ImgDetections::serializeSchema() const {
    return utility::serializeSchema(utility::getProtoMessage(this));
}

std::vector<std::uint8_t> ImgDetections::serializeProto(bool) const {
    return utility::serializeProto(utility::getProtoMessage(this));
}
#endif

}  // namespace dai
