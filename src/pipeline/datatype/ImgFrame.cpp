#define _USE_MATH_DEFINES
#include "depthai/pipeline/datatype/ImgFrame.hpp"

#include "depthai/utility/SharedMemory.hpp"
#include "spdlog/fmt/fmt.h"
#include "spdlog/spdlog.h"
#include <iostream>
#include <fstream>
#include <string>
namespace dai {


std::string ImgFrame::typeToString(Type tajp) {
    switch (tajp) {
        case Type::YUV422i: return "YUV422i";
        case Type::YUV444p: return "YUV444p";
        case Type::YUV420p: return "YUV420p";
        case Type::YUV422p: return "YUV422p";
        case Type::YUV400p: return "YUV400p";
        case Type::RGBA8888: return "RGBA8888";
        case Type::RGB161616: return "RGB161616";
        case Type::RGB888p: return "RGB888p";
        case Type::BGR888p: return "BGR888p";
        case Type::RGB888i: return "RGB888i";
        case Type::BGR888i: return "BGR888i";
        case Type::LUT2: return "LUT2";
        case Type::LUT4: return "LUT4";
        case Type::LUT16: return "LUT16";
        case Type::RAW16: return "RAW16";
        case Type::RAW14: return "RAW14";
        case Type::RAW12: return "RAW12";
        case Type::RAW10: return "RAW10";
        case Type::RAW8: return "RAW8";
        case Type::PACK10: return "PACK10";
        case Type::PACK12: return "PACK12";
        case Type::YUV444i: return "YUV444i";
        case Type::NV12: return "NV12";
        case Type::NV21: return "NV21";
        case Type::BITSTREAM: return "BITSTREAM";
        case Type::HDR: return "HDR";
        case Type::RGBF16F16F16p: return "RGBF16F16F16p";
        case Type::BGRF16F16F16p: return "BGRF16F16F16p";
        case Type::RGBF16F16F16i: return "RGBF16F16F16i";
        case Type::BGRF16F16F16i: return "BGRF16F16F16i";
        case Type::GRAY8: return "GRAY8";
        case Type::GRAYF16: return "GRAYF16";
        case Type::RAW32: return "RAW32";
        case Type::NONE: return "NONE";
        default: return "UNKNOWN";
    }
}

std::string ImgFrame::transformationToString(ImgTransformation::Transformation tajp) {
    switch (tajp) {
        case ImgTransformation::Transformation::INIT: return "INIT";
        case ImgTransformation::Transformation::CROP: return "CROP";
        case ImgTransformation::Transformation::ROTATION: return "ROTATION";
        case ImgTransformation::Transformation::PAD: return "PAD";
        case ImgTransformation::Transformation::FLIP: return "FLIP";
        case ImgTransformation::Transformation::SCALE: return "SCALE";
        default: return "UNKNOWN";
    }
}

void ImgFrame::print_for_test() {
    //buffer::ts
    std::cout << "ts.sec: " << ts.sec << std::endl;
    std::cout << "ts.nsec: " << ts.nsec << std::endl;
    
    //buffer::tsDevice
    std::cout << "tsDevice.sec: " << this->tsDevice.sec << std::endl;
    std::cout << "tsDevice.nsec: " << this->tsDevice.nsec << std::endl;
    std::cout << "sequenceNum: " << this->sequenceNum << std::endl;
    std::cout << "HFovDegrees: " << this->HFovDegrees << std::endl;
    std::cout << "category: " << this->category << std::endl;
    std::cout << "instanceNum: " << this->instanceNum << std::endl;

    //fb
    std::cout << "fb.type: " << typeToString(this->fb.type) << std::endl;
    std::cout << "fb.width: " << this->fb.width << std::endl;
    std::cout << "fb.height: " << this->fb.height << std::endl;
    std::cout << "fb.stride: " << this->fb.stride << std::endl;
    std::cout << "fb.bytesPP: " << this->fb.bytesPP << std::endl;
    std::cout << "fb.p1Offset: " << this->fb.p1Offset << std::endl;
    std::cout << "fb.p2Offset: " << this->fb.p2Offset << std::endl;
    std::cout << "fb.p3Offset: " << this->fb.p3Offset << std::endl;

    //sourceFb
    std::cout << "sourceFb.type: " << typeToString(this->sourceFb.type) << std::endl;
    std::cout << "sourceFb.width: " << this->sourceFb.width << std::endl;
    std::cout << "sourceFb.height: " << this->sourceFb.height << std::endl;
    std::cout << "sourceFb.stride: " << this->sourceFb.stride << std::endl;
    std::cout << "sourceFb.bytesPP: " << this->sourceFb.bytesPP << std::endl;
    std::cout << "sourceFb.p1Offset: " << this->sourceFb.p1Offset << std::endl;
    std::cout << "sourceFb.p2Offset: " << this->sourceFb.p2Offset << std::endl;
    std::cout << "sourceFb.p3Offset: " << this->sourceFb.p3Offset << std::endl;

    //cam
    std::cout << "cam.exposureTimeUs: " << this->cam.exposureTimeUs << std::endl;
    std::cout << "cam.sensitivityIso: " << this->cam.sensitivityIso << std::endl;
    std::cout << "cam.lensPosition: " << this->cam.lensPosition << std::endl;
    std::cout << "cam.wbColorTemp: " << this->cam.wbColorTemp << std::endl;
    std::cout << "cam.lensPositionRaw: " << this->cam.lensPositionRaw << std::endl;

    //transformations
    std::cout << "transformations.invalidFlag: " << this->transformations.invalidFlag << std::endl;
    std::cout << "transformations.transformations.size(): " << this->transformations.transformations.size() << std::endl;
    for (int i = 0; i < this->transformations.transformations.size(); i++) {
        std::cout << "transformation: " << i << std::endl;
        ImgTransformation* ImgTransformation = &(this->transformations.transformations[i]);
        std::cout << "type: " << transformationToString(ImgTransformation->transformationType) << std::endl;
        std::cout << "topLeftCropX: " << ImgTransformation->topLeftCropX << std::endl;
        std::cout << "topLeftCropY: " << ImgTransformation->topLeftCropY << std::endl;
        std::cout << "bottomRightCropX: " << ImgTransformation->bottomRightCropX << std::endl;
        std::cout << "bottomRightCropY: " << ImgTransformation->bottomRightCropY << std::endl;
        std::cout << "topPadding: " << ImgTransformation->topPadding << std::endl;
        std::cout << "bottomPadding: " << ImgTransformation->bottomPadding << std::endl;
        std::cout << "leftPadding: " << ImgTransformation->leftPadding << std::endl;
        std::cout << "rightPadding: " << ImgTransformation->rightPadding << std::endl;
        for (int j = 0; j < ImgTransformation->transformationMatrix.size(); j++) {
            for (int k = 0; k < ImgTransformation->transformationMatrix[j].size(); k++) {
                std::cout << "transformationMatrix[" << j << "][" << k << "]:" << ImgTransformation->transformationMatrix[j][k] << " ";
            }
        }
        for (int j = 0; j < ImgTransformation->invTransformationMatrix.size(); j++) {
            for (int k = 0; k < ImgTransformation->invTransformationMatrix[j].size(); k++) {
                std::cout << "invTransformationMatrix[" << j << "][" << k << "]:" << ImgTransformation->invTransformationMatrix[j][k] << " ";
            }
        }
        std::cout << "afterTransformWidth: " << ImgTransformation->afterTransformWidth << std::endl;
        std::cout << "afterTransformHeight: " << ImgTransformation->afterTransformHeight << std::endl;
        std::cout << "beforeTransformWidth: " << ImgTransformation->beforeTransformWidth << std::endl;
        std::cout << "beforeTransformHeight: " << ImgTransformation->beforeTransformHeight << std::endl;
    }

    auto dataTmp = this->getData();
    std::ofstream myfile("tmp.bin", std::ios::binary);
    if (myfile.is_open()) {
        myfile.write(reinterpret_cast<const char*>(dataTmp.data()), dataTmp.size());
        myfile.close();
    }

    /*
    std::ofstream myfile("tmp.txt");
    if (myfile.is_open()) {
        for (int i = 0; i < dataTmp.size(); i++) {
            myfile << "data[" << i << "]: " << dataTmp[i] << std::endl;
        }
        myfile.close();
    }
    */
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
    transformations.addInitTransformation(width, height);
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

Point2f ImgFrame::remapPointFromSource(const Point2f& point) const {
    if(point.isNormalized()) {
        throw std::runtime_error("Point must be denormalized");
    }
    Point2f transformedPoint = point;
    bool isClipped = false;
    for(auto& transformation : transformations.transformations) {
        transformedPoint = ImgTransformation::transformPoint(transformation, transformedPoint, isClipped);
    }
    return transformedPoint;
}

Point2f ImgFrame::remapPointToSource(const Point2f& point) const {
    if(point.isNormalized()) {
        throw std::runtime_error("Point must be denormalized");
    }
    Point2f transformedPoint = point;
    bool isClipped = false;
    // Do the loop in reverse order
    for(auto it = transformations.transformations.rbegin(); it != transformations.transformations.rend(); ++it) {
        transformedPoint = ImgTransformation::invTransformPoint(*it, transformedPoint, isClipped);
    }
    return transformedPoint;
}

Rect ImgFrame::remapRectFromSource(const Rect& rect) const {
    bool isNormalized = rect.isNormalized();
    auto returnRect = rect;
    if(isNormalized) {
        returnRect = returnRect.denormalize(getSourceWidth(), getSourceHeight());
    }
    auto topLeftTransformed = remapPointFromSource(returnRect.topLeft());
    auto bottomRightTransformed = remapPointFromSource(returnRect.bottomRight());
    returnRect = Rect(topLeftTransformed, bottomRightTransformed);
    if(isNormalized) {
        returnRect = returnRect.normalize(getWidth(), getHeight());
    }
    return returnRect;
}

Rect ImgFrame::remapRectToSource(const Rect& rect) const {
    bool isNormalized = rect.isNormalized();
    auto returnRect = rect;
    if(isNormalized) {
        returnRect = returnRect.denormalize(getWidth(), getHeight());
    }
    auto topLeftTransformed = remapPointToSource(returnRect.topLeft());
    auto bottomRightTransformed = remapPointToSource(returnRect.bottomRight());

    returnRect = Rect(topLeftTransformed, bottomRightTransformed);
    if(isNormalized) {
        returnRect = returnRect.normalize(getSourceWidth(), getSourceHeight());
    }
    return returnRect;
}

ImgFrame& ImgFrame::setSourceHFov(float degrees) {
    HFovDegrees = degrees;
    return *this;
}

float ImgFrame::getSourceHFov() const {
    return HFovDegrees;
}

float ImgFrame::getSourceDFov() const {
    // TODO only works rectlinear lenses (rectified frames).
    // Calculate the vertical FOV from the source dimensions and the source DFov
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
    // TODO only works rectlinear lenses (rectified frames).
    // Calculate the vertical FOV from the source dimensions and the source DFov
    float sourceWidth = getSourceWidth();
    float sourceHeight = getSourceHeight();

    if(sourceHeight <= 0) {
        throw std::runtime_error(fmt::format("Source height is invalid. Height: {}", sourceHeight));
    }
    if(sourceWidth <= 0) {
        throw std::runtime_error(fmt::format("Source width is invalid. Width: {}", sourceWidth));
    }
    float HFovDegrees = getSourceHFov();

    // Validate the horizontal FOV
    if(HFovDegrees <= 0 || HFovDegrees >= 180) {
        throw std::runtime_error(fmt::format("Horizontal FOV is invalid. Horizontal FOV: {}", HFovDegrees));
    }

    float HFovRadians = HFovDegrees * (static_cast<float>(M_PI) / 180.0f);

    // Calculate the tangent of half of the HFOV
    float tanHFovHalf = std::tan(HFovRadians / 2);

    // Calculate the tangent of half of the VFOV
    float tanVerticalFovHalf = (sourceHeight / sourceWidth) * tanHFovHalf;

    // Calculate the VFOV in radians
    float verticalFovRadians = 2 * std::atan(tanVerticalFovHalf);

    // Convert VFOV to degrees
    float verticalFovDegrees = verticalFovRadians * (180.0f / static_cast<float>(M_PI));
    return verticalFovDegrees;
}

bool ImgFrame::validateTransformations() const {
    if(!transformations.validateTransformationSizes()) {
        spdlog::warn("Transformation sizes are invalid");
        return false;
    }

    // Initial transformation always has to be set
    if(transformations.transformations.size() == 0) {
        spdlog::warn("No transformations set");
        return false;
    }

    if(getSourceHeight() != transformations.transformations[0].beforeTransformHeight
       || getSourceWidth() != transformations.transformations[0].beforeTransformWidth) {
        spdlog::warn("Initial transformation size is {}x{} - while source image size is {}x{}",
                     transformations.transformations[0].beforeTransformWidth,
                     transformations.transformations[0].beforeTransformHeight,
                     getSourceWidth(),
                     getSourceHeight());
        return false;
    }

    if(getHeight() != transformations.getLastHeight() || getWidth() != transformations.getLastWidth()) {
        spdlog::warn("Last transformation size is {}x{} while current transformation size is {}x{}",
                     transformations.getLastWidth(),
                     transformations.getLastHeight(),
                     getWidth(),
                     getHeight());
        return false;
    }

    return true;
}

Point2f ImgFrame::remapPointBetweenSourceFrames(const Point2f& point, const ImgFrame& sourceImage, const ImgFrame& destImage) {
    auto hFovDegreeDest = destImage.getSourceHFov();
    auto vFovDegreeDest = destImage.getSourceVFov();
    auto hFovDegreeOrigin = sourceImage.getSourceHFov();
    auto vFovDegreeOrigin = sourceImage.getSourceVFov();

    float hFovRadiansDest = (hFovDegreeDest * ((float)M_PI / 180.0f));
    float vFovRadiansDest = (vFovDegreeDest * ((float)M_PI / 180.0f));
    float hFovRadiansOrigin = (hFovDegreeOrigin * ((float)M_PI / 180.0f));
    float vFovRadiansOrigin = (vFovDegreeOrigin * ((float)M_PI / 180.0f));
    if(point.isNormalized()) {
        throw std::runtime_error("Point is normalized. Cannot remap normalized points");
    }

    if(sourceImage.getSourceWidth() == 0 || sourceImage.getSourceHeight() == 0 || destImage.getSourceWidth() == 0 || destImage.getSourceHeight() == 0) {
        throw std::runtime_error("Source image has invalid dimensions - all dimensions need to be set before remapping");
    }

    if(!(sourceImage.getSourceHFov() > 0)) {
        throw std::runtime_error("Source image has invalid horizontal FOV - horizontal FOV needs to be set before remapping");
    }

    if(!(destImage.getSourceHFov() > 0)) {
        throw std::runtime_error("Destination image has invalid horizontal FOV - horizontal FOV needs to be set before remapping");
    }

    // Calculate the factor between the FOVs
    // kX of 1.2 would mean that the destination image has 1.2 times wider FOV than the source image
    float kX = ((std::tan(hFovRadiansDest / 2) / std::tan(hFovRadiansOrigin / 2)));
    float kY = ((std::tan(vFovRadiansDest / 2) / std::tan(vFovRadiansOrigin / 2)));

    auto returnPoint = point;

    // Scale the point to the destination image
    returnPoint.x = std::round(point.x * (static_cast<float>(destImage.getSourceWidth()) / sourceImage.getSourceWidth()));
    returnPoint.y = std::round(point.y * (static_cast<float>(destImage.getSourceHeight()) / sourceImage.getSourceHeight()));

    // Adjust the point to the destination image
    unsigned adjustedWidth = std::round(destImage.getSourceWidth() * kX);
    unsigned adjustedHeight = std::round(destImage.getSourceHeight() * kY);

    int diffX = (adjustedWidth - destImage.getSourceWidth()) / 2;
    int diffY = (adjustedHeight - destImage.getSourceHeight()) / 2;

    int adjustedFrameX = returnPoint.x + diffX;
    int adjustedFrameY = returnPoint.y + diffY;

    // Scale the point back to the destination frame
    returnPoint = Point2f(std::round(adjustedFrameX / kX), std::round(adjustedFrameY / kY));
    bool pointClipped = false;
    returnPoint = ImgTransformation::clipPoint(returnPoint, destImage.getSourceWidth(), destImage.getSourceHeight(), pointClipped);

    return returnPoint;
}

Point2f ImgFrame::remapPointBetweenFrames(const Point2f& originPoint, const ImgFrame& originFrame, const ImgFrame& destFrame) {
    // First get the origin to the origin image
    // For example if this is a RGB image that was cropped and rotated and the detection was done there,
    // you remap it back as it was taken on the camera
    Point2f transformedPoint = originPoint;
    transformedPoint = originFrame.remapPointToSource(transformedPoint);
    if(originFrame.getInstanceNum() != destFrame.getInstanceNum()) {
        transformedPoint = remapPointBetweenSourceFrames(transformedPoint, originFrame, destFrame);
    } else {
        if((originFrame.getSourceHeight() != destFrame.getSourceHeight()) || (originFrame.getSourceWidth() != destFrame.getSourceWidth())
           || (originFrame.getSourceHFov() != destFrame.getSourceHFov()) || (originFrame.getSourceVFov() != destFrame.getSourceVFov())) {
            throw std::runtime_error("Frames have the same instance numbers, but different source dimensions and/or FOVs.");
        }
    }
    transformedPoint = destFrame.remapPointFromSource(transformedPoint);

    return transformedPoint;
}

Rect ImgFrame::remapRectBetweenFrames(const Rect& originRect, const ImgFrame& originFrame, const ImgFrame& destFrame) {
    bool normalized = originRect.isNormalized();
    auto returnRect = originRect;
    returnRect = returnRect.denormalize(originFrame.getWidth(), originFrame.getHeight());
    auto topLeftTransformed = remapPointBetweenFrames(returnRect.topLeft(), originFrame, destFrame);
    auto bottomRightTransformed = remapPointBetweenFrames(returnRect.bottomRight(), originFrame, destFrame);
    returnRect = Rect{topLeftTransformed, bottomRightTransformed};
    if(normalized) {
        returnRect = returnRect.normalize(destFrame.getWidth(), destFrame.getHeight());
    }
    return returnRect;
}

}  // namespace dai
