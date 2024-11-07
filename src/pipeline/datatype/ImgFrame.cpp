#define _USE_MATH_DEFINES
#include "depthai/pipeline/datatype/ImgFrame.hpp"

#include "depthai/common/ImgTransformations.hpp"
#include "depthai/common/RotatedRect.hpp"
#include "depthai/utility/SharedMemory.hpp"
#ifdef DEPTHAI_ENABLE_PROTOBUF
    #include "../../utility/ProtoSerialize.hpp"
    #include "depthai/schemas/ImgFrame.pb.h"
#endif
namespace dai {

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
std::unique_ptr<google::protobuf::Message> getProtoMessage(const ImgFrame* frame) {
    // create and populate ImgFrame protobuf message
    auto imgFrame = std::make_unique<proto::img_frame::ImgFrame>();
    proto::common::Timestamp* ts = imgFrame->mutable_ts();
    ts->set_sec(frame->ts.sec);
    ts->set_nsec(frame->ts.nsec);
    proto::common::Timestamp* tsDevice = imgFrame->mutable_tsdevice();
    tsDevice->set_sec(frame->tsDevice.sec);
    tsDevice->set_nsec(frame->tsDevice.nsec);

    imgFrame->set_sequencenum(frame->sequenceNum);

    proto::img_frame::Specs* fb = imgFrame->mutable_fb();
    fb->set_type(static_cast<proto::img_frame::Type>(frame->fb.type));
    fb->set_width(frame->fb.width);
    fb->set_height(frame->fb.height);
    fb->set_stride(frame->fb.stride);
    fb->set_bytespp(frame->fb.bytesPP);
    fb->set_p1offset(frame->fb.p1Offset);
    fb->set_p2offset(frame->fb.p2Offset);
    fb->set_p3offset(frame->fb.p3Offset);

    proto::img_frame::Specs* sourceFb = imgFrame->mutable_sourcefb();
    sourceFb->set_type(static_cast<proto::img_frame::Type>(frame->sourceFb.type));
    sourceFb->set_width(frame->sourceFb.width);
    sourceFb->set_height(frame->sourceFb.height);
    sourceFb->set_stride(frame->sourceFb.stride);
    sourceFb->set_bytespp(frame->sourceFb.bytesPP);
    sourceFb->set_p1offset(frame->sourceFb.p1Offset);
    sourceFb->set_p2offset(frame->sourceFb.p2Offset);
    sourceFb->set_p3offset(frame->sourceFb.p3Offset);

    proto::common::CameraSettings* cam = imgFrame->mutable_cam();
    cam->set_exposuretimeus(frame->cam.exposureTimeUs);
    cam->set_sensitivityiso(frame->cam.sensitivityIso);
    cam->set_lensposition(frame->cam.lensPosition);
    cam->set_wbcolortemp(frame->cam.wbColorTemp);
    cam->set_lenspositionraw(frame->cam.lensPositionRaw);

    imgFrame->set_instancenum(frame->instanceNum);

    imgFrame->set_category(frame->category);

    imgFrame->set_data(frame->data->getData().data(), frame->data->getData().size());
    return imgFrame;
}

utility::ProtoSerializable::SchemaPair ImgFrame::serializeSchema() const {
    return utility::serializeSchema(getProtoMessage(this));
}

std::vector<std::uint8_t> ImgFrame::serializeProto() const {
    return utility::serializeProto(getProtoMessage(this));
}
#endif
}  // namespace dai
