#define _USE_MATH_DEFINES
#include "depthai/pipeline/datatype/ImgFrame.hpp"

#include "depthai/common/ImgTransformations.hpp"
#include "depthai/common/RotatedRect.hpp"
#include "depthai/utility/SharedMemory.hpp"

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
    float fx = transformation.getSourceIntrinsicMatrix()[0][0];

    // Calculate vertical FoV (in radians)
    float horizontalFoV = 2 * atan(getWidth() / (2.0f * fx));

    // Convert radians to degrees
    return horizontalFoV * 180.0f / (float)M_PI;
}

float ImgFrame::getSourceDFov() const {
    float sourceWidth = getSourceWidth();
    float sourceHeight = getSourceHeight();

    if(sourceHeight <= 0) {
        throw std::runtime_error(fmt::format("Source height is invalid. Height: {}", sourceHeight));
    }
    if(sourceWidth <= 0) {
        throw std::runtime_error(fmt::format("Source width is invalid. Width: {}", sourceWidth));
    }
    float HFovDegrees = getSourceHFov();

    // Calculate the diagonal ratio (DR)
    float dr = std::sqrt(std::pow(sourceWidth, 2) + std::pow(sourceHeight, 2));

    // Validate the horizontal FOV
    if(HFovDegrees <= 0 || HFovDegrees >= 180) {
        throw std::runtime_error(fmt::format("Horizontal FOV is invalid. Horizontal FOV: {}", HFovDegrees));
    }

    float HFovRadians = HFovDegrees * (static_cast<float>(M_PI) / 180.0f);

    // Calculate the tangent of half of the HFOV
    float tanHFovHalf = std::tan(HFovRadians / 2);

    // Calculate the tangent of half of the VFOV
    float tanDiagonalFovHalf = (dr / sourceWidth) * tanHFovHalf;

    // Calculate the VFOV in radians
    float diagonalFovRadians = 2 * std::atan(tanDiagonalFovHalf);

    // Convert VFOV to degrees
    float diagonalFovDegrees = diagonalFovRadians * (180.0f / static_cast<float>(M_PI));
    return diagonalFovDegrees;
}

float ImgFrame::getSourceVFov() const {
    float fy = transformation.getSourceIntrinsicMatrix()[1][1];

    // Calculate vertical FoV (in radians)
    float verticalFoV = 2 * atan(getHeight() / (2.0f * fy));

    // Convert radians to degrees
    return verticalFoV * 180.0f / (float)M_PI;
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

std::unique_ptr<google::protobuf::Message> ImgFrame::getProtoMessage() const {
    // create and populate ImgFrame protobuf message
    auto imgFrame = std::make_unique<proto::img_frame::ImgFrame>();
    proto::common::Timestamp* ts = imgFrame->mutable_ts();
    ts->set_sec(this->ts.sec);
    ts->set_nsec(this->ts.nsec);
    proto::common::Timestamp* tsDevice = imgFrame->mutable_tsdevice();
    tsDevice->set_sec(this->tsDevice.sec);
    tsDevice->set_nsec(this->tsDevice.nsec);

    imgFrame->set_sequencenum(this->sequenceNum);

    proto::img_frame::Specs* fb = imgFrame->mutable_fb();
    fb->set_type(static_cast<proto::img_frame::Type>(this->fb.type));
    fb->set_width(this->fb.width);
    fb->set_height(this->fb.height);
    fb->set_stride(this->fb.stride);
    fb->set_bytespp(this->fb.bytesPP);
    fb->set_p1offset(this->fb.p1Offset);
    fb->set_p2offset(this->fb.p2Offset);
    fb->set_p3offset(this->fb.p3Offset);

    proto::img_frame::Specs* sourceFb = imgFrame->mutable_sourcefb();
    sourceFb->set_type(static_cast<proto::img_frame::Type>(this->sourceFb.type));
    sourceFb->set_width(this->sourceFb.width);
    sourceFb->set_height(this->sourceFb.height);
    sourceFb->set_stride(this->sourceFb.stride);
    sourceFb->set_bytespp(this->sourceFb.bytesPP);
    sourceFb->set_p1offset(this->sourceFb.p1Offset);
    sourceFb->set_p2offset(this->sourceFb.p2Offset);
    sourceFb->set_p3offset(this->sourceFb.p3Offset);

    proto::common::CameraSettings* cam = imgFrame->mutable_cam();
    cam->set_exposuretimeus(this->cam.exposureTimeUs);
    cam->set_sensitivityiso(this->cam.sensitivityIso);
    cam->set_lensposition(this->cam.lensPosition);
    cam->set_wbcolortemp(this->cam.wbColorTemp);
    cam->set_lenspositionraw(this->cam.lensPositionRaw);

    imgFrame->set_instancenum(this->instanceNum);

    imgFrame->set_category(this->category);

    proto::common::ImgTransformation* imgTransformation = imgFrame->mutable_transformation();
    const auto [width, height] = this->transformation.getSize();
    const auto [srcWidth, srcHeight] = this->transformation.getSourceSize();
    imgTransformation->set_width(width);
    imgTransformation->set_height(height);
    imgTransformation->set_srcwidth(srcWidth);
    imgTransformation->set_srcheight(srcHeight);

    proto::common::TransformationMatrix* transformationMatrix = imgTransformation->mutable_transformationmatrix();
    for(const auto& array : transformation.getMatrix()) {
        proto::common::FloatArray* floatArray = transformationMatrix->add_arrays();
        for(const auto& value : array) {
            floatArray->add_values(value);
        }
    }
    proto::common::TransformationMatrix* sourceIntrinsicMatrix = imgTransformation->mutable_sourceintrinsicmatrix();
    for(const auto& array : transformation.getSourceIntrinsicMatrix()) {
        proto::common::FloatArray* floatArray = sourceIntrinsicMatrix->add_arrays();
        for(const auto& value : array) {
            floatArray->add_values(value);
        }
    }

    imgTransformation->set_distortionmodel(static_cast<proto::common::CameraModel>(this->transformation.getDistortionModel()));
    proto::common::FloatArray* distortionCoefficients = imgTransformation->mutable_distortioncoefficients();
    for(const auto& value : this->transformation.getDistortionCoefficients()) {
        distortionCoefficients->add_values(value);
    }

    imgFrame->set_data(this->data->getData().data(), this->data->getData().size());
    return imgFrame;
}

void ImgFrame::setProtoMessage(const std::unique_ptr<google::protobuf::Message> msg) {
    auto imgFrame = dynamic_cast<proto::img_frame::ImgFrame*>(msg.get());
    // create and populate ImgFrame protobuf message
    this->setTimestamp(utility::fromProtoTimestamp(imgFrame->ts()));
    this->setTimestampDevice(utility::fromProtoTimestamp(imgFrame->tsdevice()));

    this->setSequenceNum(imgFrame->sequencenum());

    this->fb.type = static_cast<Type>(imgFrame->fb().type());
    this->fb.width = imgFrame->fb().width();
    this->fb.height = imgFrame->fb().height();
    this->fb.stride = imgFrame->fb().stride();
    this->fb.bytesPP = imgFrame->fb().bytespp();
    this->fb.p1Offset = imgFrame->fb().p1offset();
    this->fb.p2Offset = imgFrame->fb().p2offset();
    this->fb.p3Offset = imgFrame->fb().p3offset();

    this->sourceFb.type = static_cast<Type>(imgFrame->sourcefb().type());
    this->sourceFb.width = imgFrame->sourcefb().width();
    this->sourceFb.height = imgFrame->sourcefb().height();
    this->sourceFb.stride = imgFrame->sourcefb().stride();
    this->sourceFb.bytesPP = imgFrame->sourcefb().bytespp();
    this->sourceFb.p1Offset = imgFrame->sourcefb().p1offset();
    this->sourceFb.p2Offset = imgFrame->sourcefb().p2offset();
    this->sourceFb.p3Offset = imgFrame->sourcefb().p3offset();

    this->cam.exposureTimeUs = imgFrame->cam().exposuretimeus();
    this->cam.sensitivityIso = imgFrame->cam().sensitivityiso();
    this->cam.lensPosition = imgFrame->cam().lensposition();
    this->cam.wbColorTemp = imgFrame->cam().wbcolortemp();
    this->cam.lensPositionRaw = imgFrame->cam().lenspositionraw();

    this->instanceNum = imgFrame->instancenum();

    this->category = imgFrame->category();

    std::array<std::array<float, 3>, 3> transformationMatrix;
    std::array<std::array<float, 3>, 3> sourceIntrinsicMatrix;
    std::vector<float> distortionCoefficients;
    distortionCoefficients.reserve(imgFrame->transformation().distortioncoefficients().values_size());
    for(auto i = 0U; i < 3; ++i)
        for(auto j = 0U; j < 3; ++j) transformationMatrix[i][j] = imgFrame->transformation().transformationmatrix().arrays(i).values(j);
    for(auto i = 0U; i < 3; ++i)
        for(auto j = 0U; j < 3; ++j) sourceIntrinsicMatrix[i][j] = imgFrame->transformation().sourceintrinsicmatrix().arrays(i).values(j);
    for(auto i = 0; i < imgFrame->transformation().distortioncoefficients().values_size(); ++i)
        distortionCoefficients.push_back(imgFrame->transformation().distortioncoefficients().values(i));

    this->transformation = ImgTransformation(imgFrame->transformation().srcwidth(),
                                             imgFrame->transformation().srcheight(),
                                             sourceIntrinsicMatrix,
                                             static_cast<CameraModel>(imgFrame->transformation().distortionmodel()),
                                             distortionCoefficients);
    this->transformation.addTransformation(transformationMatrix);
    this->transformation.addCrop(0, 0, imgFrame->transformation().width(), imgFrame->transformation().height());

    std::vector<uint8_t> data(imgFrame->data().begin(), imgFrame->data().end());
    this->setData(data);
}

}  // namespace dai
