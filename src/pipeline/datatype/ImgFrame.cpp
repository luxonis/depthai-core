#define _USE_MATH_DEFINES
#include "depthai/pipeline/datatype/ImgFrame.hpp"

#include "depthai/common/ImgTransformations.hpp"
#include "depthai/common/RotatedRect.hpp"
#include "depthai/utility/SharedMemory.hpp"
#ifdef DEPTHAI_ENABLE_PROTOBUF
    #include "depthai/schemas/ImgFrame.pb.h"
    #include "utility/ProtoSerialize.hpp"
#endif
namespace dai {

ImgFrame::~ImgFrame() = default;

void ImgFrame::serialize(std::vector<std::uint8_t>& metadata, DatatypeEnum& datatype) const {
    metadata = utility::serialize(*this);
    datatype = DatatypeEnum::ImgFrame;
}

ImgFrame::ImgFrame() {
    // Set timestamp to now
    setTimestamp(std::chrono::steady_clock::now());
}

ImgFrame::ImgFrame(size_t size) : ImgFrame() {
    auto mem = std::make_shared<VectorMemory>();
    mem->resize(size);
    data = mem;
}

ImgFrame::ImgFrame(long fd) : ImgFrame() {
    auto mem = std::make_shared<SharedMemory>(fd);
    data = mem;
}

ImgFrame::ImgFrame(long fd, size_t size) : ImgFrame() {
    auto mem = std::make_shared<SharedMemory>(fd, size);
    data = mem;
}

std::chrono::time_point<std::chrono::steady_clock, std::chrono::steady_clock::duration> ImgFrame::getTimestamp(CameraExposureOffset offset) const {
    auto ts = getTimestamp();
    auto expTime = getExposureTime();
    switch(offset) {
        case CameraExposureOffset::START:
            return ts - expTime;
        case CameraExposureOffset::MIDDLE:
            return ts - expTime / 2;
        case CameraExposureOffset::END:
        default:
            return ts;
    }
}
std::chrono::time_point<std::chrono::steady_clock, std::chrono::steady_clock::duration> ImgFrame::getTimestampDevice(CameraExposureOffset offset) const {
    auto ts = getTimestampDevice();
    auto expTime = getExposureTime();
    switch(offset) {
        case CameraExposureOffset::START:
            return ts - expTime;
        case CameraExposureOffset::MIDDLE:
            return ts - expTime / 2;
        case CameraExposureOffset::END:
        default:
            return ts;
    }
}

unsigned int ImgFrame::getInstanceNum() const {
    return instanceNum;
}
unsigned int ImgFrame::getCategory() const {
    return category;
}
unsigned int ImgFrame::getWidth() const {
    return fb.width;
}

unsigned int ImgFrame::getStride() const {
    if(fb.stride == 0) {
        return static_cast<unsigned>(std::round(static_cast<float>(getWidth()) * getBytesPerPixel()));
    }
    return fb.stride;
}
unsigned int ImgFrame::getPlaneStride(int planeIndex) const {
    int planeStride = 0;
    switch(planeIndex) {
        case 0:
            planeStride = fb.p2Offset - fb.p1Offset;
            break;
        case 1:
            planeStride = fb.p3Offset - fb.p2Offset;
            break;
    }
    if(planeStride <= 0) planeStride = getStride() * getHeight();
    return planeStride;
}
unsigned int ImgFrame::getHeight() const {
    return fb.height;
}
unsigned int ImgFrame::getPlaneHeight() const {
    return getPlaneStride() / getStride();
}
ImgFrame::Type ImgFrame::getType() const {
    return fb.type;
}
float ImgFrame::getBytesPerPixel() const {
    return typeToBpp(getType());
}
std::chrono::microseconds ImgFrame::getExposureTime() const {
    return std::chrono::microseconds(cam.exposureTimeUs);
}
int ImgFrame::getSensitivity() const {
    return cam.sensitivityIso;
}
int ImgFrame::getColorTemperature() const {
    return cam.wbColorTemp;
}
int ImgFrame::getLensPosition() const {
    return cam.lensPosition;
}

unsigned int ImgFrame::getSourceWidth() const {
    return sourceFb.width;
}

float ImgFrame::getLensPositionRaw() const {
    return cam.lensPositionRaw;
}

unsigned int ImgFrame::getSourceHeight() const {
    return sourceFb.height;
}

ImgFrame& ImgFrame::setInstanceNum(unsigned int instanceNum) {
    this->instanceNum = instanceNum;
    return *this;
}
ImgFrame& ImgFrame::setCategory(unsigned int category) {
    this->category = category;
    return *this;
}

ImgFrame& ImgFrame::setWidth(unsigned int width) {
    fb.width = width;
    return *this;
}
ImgFrame& ImgFrame::setStride(unsigned int stride) {
    fb.stride = stride;
    return *this;
}
ImgFrame& ImgFrame::setHeight(unsigned int height) {
    fb.height = height;
    return *this;
}
ImgFrame& ImgFrame::setSize(unsigned int width, unsigned int height) {
    setWidth(width);
    setHeight(height);
    return *this;
}
ImgFrame& ImgFrame::setSize(std::tuple<unsigned int, unsigned int> size) {
    setSize(std::get<0>(size), std::get<1>(size));
    return *this;
}

ImgFrame& ImgFrame::setSourceSize(unsigned int width, unsigned int height) {
    sourceFb.width = width;
    sourceFb.stride = width;
    sourceFb.height = height;
    transformation = ImgTransformation(width, height);
    return *this;
}

ImgFrame& ImgFrame::setSourceSize(std::tuple<unsigned int, unsigned int> size) {
    setSourceSize(std::get<0>(size), std::get<1>(size));
    return *this;
}
ImgFrame& ImgFrame::setType(Type type) {
    fb.type = type;
    fb.bytesPP = ImgFrame::typeToBpp(fb.type);
    return *this;
}

ImgFrame& ImgFrame::setMetadata(const ImgFrame& sourceFrame) {
    auto tmpData = this->data;
    *this = sourceFrame;
    this->data = tmpData;  // Keep the original data
    return *this;
}

ImgFrame& ImgFrame::setMetadata(const std::shared_ptr<ImgFrame>& sourceFrame) {
    if(sourceFrame == nullptr) {
        throw std::invalid_argument("Source frame is null");
    }
    return setMetadata(*sourceFrame);
}

ImgFrame& ImgFrame::copyDataFrom(const ImgFrame& sourceFrame) {
    std::vector<uint8_t> data(sourceFrame.data->getData().begin(), sourceFrame.data->getData().end());
    setData(std::move(data));
    return *this;
}

ImgFrame& ImgFrame::copyDataFrom(const std::shared_ptr<ImgFrame>& sourceFrame) {
    return copyDataFrom(*sourceFrame);
}

std::shared_ptr<ImgFrame> ImgFrame::clone() const {
    auto clone = std::make_shared<ImgFrame>();
    clone->setMetadata(*this);
    clone->copyDataFrom(*this);
    return clone;
}

bool ImgFrame::validateTransformations() const {
    const auto [width, height] = transformation.getSize();
    const auto [srcWidth, srcHeight] = transformation.getSourceSize();
    return transformation.isValid() && width == getWidth() && height == getHeight() && srcWidth == getSourceWidth() && srcHeight == getSourceHeight();
}

Point2f ImgFrame::remapPointFromSource(const Point2f& point) const {
    if(point.isNormalized()) {
        throw std::runtime_error("Point must be denormalized");
    }
    if(!validateTransformations()) {
        throw std::runtime_error("ImgTransformation is not valid");
    }
    return transformation.transformPoint(point);
}

Point2f ImgFrame::remapPointToSource(const Point2f& point) const {
    if(point.isNormalized()) {
        throw std::runtime_error("Point must be denormalized");
    }
    if(!validateTransformations()) {
        throw std::runtime_error("ImgTransformation is not valid");
    }
    return transformation.invTransformPoint(point);
}

Rect ImgFrame::remapRectFromSource(const Rect& rect) const {
    bool normalized = rect.isNormalized();
    auto srcRect = rect;
    srcRect = srcRect.denormalize(getSourceWidth(), getSourceHeight());
    dai::RotatedRect srcRRect;
    srcRRect.size = {srcRect.width, srcRect.height};
    srcRRect.center = {srcRect.x + srcRect.width / 2.f, srcRect.y + srcRect.height / 2.f};
    srcRRect.angle = 0.f;
    auto dstRRect = this->transformation.transformRect(srcRRect);
    auto [minx, miny, maxx, maxy] = dstRRect.getOuterRect();
    dai::Rect returnRect((int)roundf(minx), (int)roundf(miny), (int)roundf(maxx - minx), (int)roundf(maxy - miny));
    if(normalized) {
        returnRect = returnRect.normalize(getWidth(), getHeight());
    }
    return returnRect;
}

Rect ImgFrame::remapRectToSource(const Rect& rect) const {
    bool normalized = rect.isNormalized();
    auto srcRect = rect;
    srcRect = srcRect.denormalize(getWidth(), getHeight());
    dai::RotatedRect srcRRect;
    srcRRect.size = {srcRect.width, srcRect.height};
    srcRRect.center = {srcRect.x + srcRect.width / 2.f, srcRect.y + srcRect.height / 2.f};
    srcRRect.angle = 0.f;
    auto dstRRect = this->transformation.invTransformRect(srcRRect);
    auto [minx, miny, maxx, maxy] = dstRRect.getOuterRect();
    dai::Rect returnRect((int)roundf(minx), (int)roundf(miny), (int)roundf(maxx - minx), (int)roundf(maxy - miny));
    if(normalized) {
        returnRect = returnRect.normalize(getSourceWidth(), getSourceHeight());
    }
    return returnRect;
}

float ImgFrame::getSourceHFov() const {
    return transformation.getHFov(true);
}

float ImgFrame::getSourceDFov() const {
    return transformation.getDFov(true);
}

float ImgFrame::getSourceVFov() const {
    return transformation.getVFov(true);
}

Point2f ImgFrame::remapPointBetweenFrames(const Point2f& originPoint, const ImgFrame& originFrame, const ImgFrame& destFrame) {
    if(originFrame.getInstanceNum() == destFrame.getInstanceNum()) {
        if((originFrame.getSourceHeight() != destFrame.getSourceHeight()) || (originFrame.getSourceWidth() != destFrame.getSourceWidth())
           || (originFrame.getSourceHFov() != destFrame.getSourceHFov()) || (originFrame.getSourceVFov() != destFrame.getSourceVFov())) {
            throw std::runtime_error("Frames have the same instance numbers, but different source dimensions and/or FOVs.");
        }
    }
    return originFrame.transformation.remapPointTo(destFrame.transformation, originPoint);
}

Rect ImgFrame::remapRectBetweenFrames(const Rect& originRect, const ImgFrame& originFrame, const ImgFrame& destFrame) {
    if(originFrame.getInstanceNum() == destFrame.getInstanceNum()) {
        if((originFrame.getSourceHeight() != destFrame.getSourceHeight()) || (originFrame.getSourceWidth() != destFrame.getSourceWidth())
           || (originFrame.getSourceHFov() != destFrame.getSourceHFov()) || (originFrame.getSourceVFov() != destFrame.getSourceVFov())) {
            throw std::runtime_error("Frames have the same instance numbers, but different source dimensions and/or FOVs.");
        }
    }
    bool normalized = originRect.isNormalized();
    auto srcRect = originRect;
    srcRect = srcRect.denormalize(originFrame.getWidth(), originFrame.getHeight());
    dai::RotatedRect srcRRect;
    srcRRect.size = {srcRect.width, srcRect.height};
    srcRRect.center = {srcRect.x + srcRect.width / 2.f, srcRect.y + srcRect.height / 2.f};
    srcRRect.angle = 0.f;
    auto dstRRect = originFrame.transformation.remapRectTo(destFrame.transformation, srcRRect);
    auto [minx, miny, maxx, maxy] = dstRRect.getOuterRect();
    dai::Rect returnRect((int)roundf(minx), (int)roundf(miny), (int)roundf(maxx - minx), (int)roundf(maxy - miny));
    if(normalized) {
        returnRect = returnRect.normalize(destFrame.getWidth(), destFrame.getHeight());
    }
    return returnRect;
}

#ifdef DEPTHAI_ENABLE_PROTOBUF
ProtoSerializable::SchemaPair ImgFrame::serializeSchema() const {
    return utility::serializeSchema(utility::getProtoMessage(this));
}

std::vector<std::uint8_t> ImgFrame::serializeProto(bool metadataOnly) const {
    return utility::serializeProto(utility::getProtoMessage(this, metadataOnly));
}
#endif
}  // namespace dai
