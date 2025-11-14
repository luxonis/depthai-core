#include "depthai/pipeline/node/Rectification.hpp"

#include <chrono>

#include "common/ImgTransformations.hpp"
#include "depthai/pipeline/Pipeline.hpp"
#include "pipeline/ThreadedNodeImpl.hpp"

#if defined(DEPTHAI_HAVE_OPENCV_SUPPORT)
    #include <opencv2/calib3d.hpp>
#endif

namespace dai {
namespace node {

Rectification& Rectification::setOutputSize(uint32_t width, uint32_t height) {
    properties.outputWidth = width;
    properties.outputHeight = height;
    return *this;
}

void Rectification::setRunOnHost(bool runOnHost) {
    runOnHostVar = runOnHost;
}

bool Rectification::runOnHost() const {
    return runOnHostVar;
}

#if !defined(DEPTHAI_HAVE_OPENCV_SUPPORT)
void Rectification::run() {
    throw std::runtime_error("Rectification node requires OpenCV support to run. Please enable OpenCV support in your build configuration.");
}
#else   // DEPTHAI_HAVE_OPENCV_SUPPORT

namespace {

template <typename T>
std::vector<T> flatten(const std::vector<std::vector<T> >& orig) {
    std::vector<T> ret;
    for(const auto& v : orig) ret.insert(ret.end(), v.begin(), v.end());
    return ret;
}

cv::Mat vecToCvMat(int rows, int cols, int type, const std::vector<std::vector<float> >& orig) {
    std::vector<float> flat = flatten(orig);
    cv::Mat cvMat = cv::Mat(rows, cols, type);
    memcpy(cvMat.data, flat.data(), flat.size() * sizeof(flat[0]));
    return cvMat;
}

cv::Mat vecToCvMat(int rows, int cols, int type, const std::vector<float>& orig) {
    cv::Mat cvMat = cv::Mat(rows, cols, type);
    memcpy(cvMat.data, orig.data(), orig.size() * sizeof(orig[0]));
    return cvMat;
}

cv::Mat arrayToCvMat(int rows, int cols, int type, const std::array<std::array<float, 3>, 3>& orig) {
    cv::Mat cvMat = cv::Mat(rows, cols, type);
    memcpy(cvMat.data, orig.data(), orig.size() * sizeof(orig[0]));
    return cvMat;
}

std::pair<cv::Mat, cv::Mat> computeRectificationMatrices(cv::Mat M1, cv::Mat d1, cv::Mat M2, cv::Mat d2, cv::Size imageSize, cv::Mat R, cv::Mat T) {
    cv::Mat R1, R2, P1, P2, Q;
    cv::Mat _cv_M1, _cv_M2, _cv_d1, _cv_d2, _cv_R, _cv_T;
    M1.convertTo(_cv_M1, CV_64FC1);
    M2.convertTo(_cv_M2, CV_64FC1);
    d1.convertTo(_cv_d1, CV_64FC1);
    d2.convertTo(_cv_d2, CV_64FC1);
    R.convertTo(_cv_R, CV_64FC1);
    T.convertTo(_cv_T, CV_64FC1);
    _cv_T = _cv_T.t();

    cv::stereoRectify(
        _cv_M1, _cv_d1, _cv_M2, _cv_d2, imageSize, _cv_R, _cv_T, R1, R2, P1, P2, Q);  // todo expose alpha param to user for increased fov, maybe flags?

    cv::Mat cv_R1, cv_R2;
    R1.convertTo(cv_R1, CV_32FC1);
    R2.convertTo(cv_R2, CV_32FC1);

    return std::make_pair(cv_R1, cv_R2);
}

std::string matToString(const cv::Mat& mat) {
    std::ostringstream oss;
    oss << mat;
    return oss.str();
}

}  // namespace

CalibrationHandler Rectification::getCalibrationData() const {
    if(device) {
        return device->readCalibration();
    } else {
        return getParentPipeline().getCalibrationData();
    }
}

void Rectification::run() {
    auto& logger = pimpl->logger;
    using namespace std::chrono;
    if(runOnHost()) {
        auto device = getParentPipeline().getDefaultDevice();
        if(device && device->getPlatform() != Platform::RVC4) {
            throw std::runtime_error("Rectification node is only supported on RVC4 platform");
        }
    }

    bool initialized = false;
    cv::Mat cv_rectificationMap1X, cv_rectificationMap1Y;
    cv::Mat cv_rectificationMap2X, cv_rectificationMap2Y;

    cv::Mat cv_targetCameraMatrix1, cv_targetCameraMatrix2;
    std::array<std::array<float, 3>, 3> targetM1, targetM2;
    dai::ImgTransformation output1ImgTransformation;
    dai::ImgTransformation output2ImgTransformation;
    while(isRunning()) {
        auto input1Frame = input1.get<dai::ImgFrame>();
        auto input2Frame = input2.get<dai::ImgFrame>();
        uint32_t output1FrameWidth;
        uint32_t output1FrameHeight;
        uint32_t output2FrameWidth;
        uint32_t output2FrameHeight;

        if(properties.outputWidth.has_value() && properties.outputHeight.has_value()) {
            output1FrameWidth = properties.outputWidth.value();
            output1FrameHeight = properties.outputHeight.value();
            output2FrameWidth = properties.outputWidth.value();
            output2FrameHeight = properties.outputHeight.value();
        } else {
            output1FrameWidth = input1Frame->getWidth();
            output1FrameHeight = input1Frame->getHeight();
            output2FrameWidth = input2Frame->getWidth();
            output2FrameHeight = input2Frame->getHeight();
        }

        if(!initialized) {
            output1ImgTransformation = input1Frame->transformation;
            output2ImgTransformation = input2Frame->transformation;
            auto calib = getCalibrationData();

            auto leftSocket = (dai::CameraBoardSocket)input1Frame->getInstanceNum();
            auto rightSocket = (dai::CameraBoardSocket)input2Frame->getInstanceNum();

            auto M1 = input1Frame->transformation.getIntrinsicMatrix();
            auto M2 = input2Frame->transformation.getIntrinsicMatrix();

            auto cv_M1 = arrayToCvMat(3, 3, CV_32FC1, M1);
            auto cv_M2 = arrayToCvMat(3, 3, CV_32FC1, M2);

            auto d1 = input1Frame->transformation.getDistortionCoefficients();
            auto d2 = input2Frame->transformation.getDistortionCoefficients();

            auto cv_d1 = vecToCvMat(1, d1.size(), CV_32FC1, d1);
            auto cv_d2 = vecToCvMat(1, d2.size(), CV_32FC1, d2);

            auto R = calib.getCameraRotationMatrix(leftSocket, rightSocket);
            auto cv_R = vecToCvMat(3, 3, CV_32FC1, R);
            auto T = calib.getCameraTranslationVector(leftSocket, rightSocket, false);
            auto cv_T = vecToCvMat(1, 3, CV_32FC1, T);

            cv::Mat cv_R1, cv_R2;
            cv::Size imageSize = cv::Size(input1Frame->getWidth(), input1Frame->getHeight());
            std::tie(cv_R1, cv_R2) = computeRectificationMatrices(cv_M1, cv_d1, cv_M2, cv_d2, imageSize, cv_R, cv_T);

            targetM1 = M1;
            targetM2 = M2;

            if(properties.outputWidth.has_value() && properties.outputHeight.has_value()) {
                auto scale1X = static_cast<float>(output1FrameWidth) / static_cast<float>(input1Frame->getWidth());
                auto scale1Y = static_cast<float>(output1FrameHeight) / static_cast<float>(input1Frame->getHeight());
                auto scale2X = static_cast<float>(output2FrameWidth) / static_cast<float>(input2Frame->getWidth());
                auto scale2Y = static_cast<float>(output2FrameHeight) / static_cast<float>(input2Frame->getHeight());
                output1ImgTransformation.addScale(scale1X, scale1Y);
                output2ImgTransformation.addScale(scale2X, scale2Y);

                targetM1 = output1ImgTransformation.getIntrinsicMatrix();
                targetM2 = output2ImgTransformation.getIntrinsicMatrix();
            }

            targetM2 = targetM1;  // TODO alignment target

            auto cv_targetCameraMatrix1 = arrayToCvMat(3, 3, CV_32FC1, targetM1);
            auto cv_targetCameraMatrix2 = arrayToCvMat(3, 3, CV_32FC1, targetM2);

            if(properties.enableRectification == false) {
                cv_R1 = cv::Mat::eye(3, 3, CV_32FC1);
                cv_R2 = cv::Mat::eye(3, 3, CV_32FC1);
                cv_d1 = cv::Mat::zeros(1, d1.size(), CV_32FC1);
                cv_d2 = cv::Mat::zeros(1, d2.size(), CV_32FC1);
            }

            cv::initUndistortRectifyMap(cv_M1,
                                        cv_d1,
                                        cv_R1,
                                        cv_targetCameraMatrix1,
                                        cv::Size(output1FrameWidth, output1FrameHeight),
                                        CV_32FC1,
                                        cv_rectificationMap1X,
                                        cv_rectificationMap1Y);
            cv::initUndistortRectifyMap(cv_M2,
                                        cv_d2,
                                        cv_R2,
                                        cv_targetCameraMatrix2,
                                        cv::Size(output2FrameWidth, output2FrameHeight),
                                        CV_32FC1,
                                        cv_rectificationMap2X,
                                        cv_rectificationMap2Y);

            logger->debug("R = {}", matToString(cv_R));
            logger->debug("T = {}", matToString(cv_T));
            logger->debug("M1 = {}", matToString(cv_M1));
            logger->debug("M2 = {}", matToString(cv_M2));
            logger->debug("R1 = {}", matToString(cv_R1));
            logger->debug("R2 = {}", matToString(cv_R2));
            logger->debug("TARGET_MATRIX1 = {}", matToString(cv_targetCameraMatrix1));
            logger->debug("TARGET_MATRIX2 = {}", matToString(cv_targetCameraMatrix2));

            initialized = true;
        }

        auto start = steady_clock::now();

        std::shared_ptr<dai::ImgFrame> rectifiedFrame1 = std::make_shared<dai::ImgFrame>();
        size_t frameSize1 = output1FrameWidth * output1FrameHeight;
        rectifiedFrame1->setData(std::vector<uint8_t>(frameSize1));

        rectifiedFrame1->setMetadata(*input1Frame);
        rectifiedFrame1->setWidth(output1FrameWidth);
        rectifiedFrame1->setHeight(output1FrameHeight);
        rectifiedFrame1->setType(dai::ImgFrame::Type::RAW8);
        rectifiedFrame1->fb.stride = rectifiedFrame1->fb.width * rectifiedFrame1->getBytesPerPixel();

        std::shared_ptr<dai::ImgFrame> rectifiedFrame2 = std::make_shared<dai::ImgFrame>();

        size_t frameSize2 = output2FrameWidth * output2FrameHeight;
        rectifiedFrame2->setData(std::vector<uint8_t>(frameSize2));

        rectifiedFrame2->setMetadata(*input2Frame);
        rectifiedFrame2->setWidth(output2FrameWidth);
        rectifiedFrame2->setHeight(output2FrameHeight);
        rectifiedFrame2->setType(dai::ImgFrame::Type::RAW8);
        rectifiedFrame2->fb.stride = rectifiedFrame2->fb.width * rectifiedFrame2->getBytesPerPixel();

        cv::Mat cv_input1 = input1Frame->getCvFrame();
        cv::Mat cv_input2 = input2Frame->getCvFrame();

        // convert to gray if needed
        if(cv_input1.channels() == 3) {
            cv::cvtColor(cv_input1, cv_input1, cv::COLOR_BGR2GRAY);
        }
        if(cv_input2.channels() == 3) {
            cv::cvtColor(cv_input2, cv_input2, cv::COLOR_BGR2GRAY);
        }

        cv::remap(
            cv_input1, rectifiedFrame1->getFrame(), cv_rectificationMap1X, cv_rectificationMap1Y, cv::INTER_LINEAR, cv::BORDER_CONSTANT, cv::Scalar(0, 0, 0));
        cv::remap(
            cv_input2, rectifiedFrame2->getFrame(), cv_rectificationMap2X, cv_rectificationMap2Y, cv::INTER_LINEAR, cv::BORDER_CONSTANT, cv::Scalar(0, 0, 0));

        rectifiedFrame1->transformation = output1ImgTransformation;
        rectifiedFrame1->transformation.setDistortionCoefficients({});

        rectifiedFrame2->transformation = output1ImgTransformation;  // Set both to same for alignment
        rectifiedFrame2->transformation.setDistortionCoefficients({});

        auto end = steady_clock::now();
        auto duration = duration_cast<milliseconds>(end - start).count();
        logger->debug("Rectification took {} ms", duration);

        output1.send(rectifiedFrame1);
        output2.send(rectifiedFrame2);

        // Passthrough the message
        passthrough1.send(input1Frame);
        passthrough2.send(input2Frame);
    }
}
#endif  // DEPTHAI_HAVE_OPENCV_SUPPORT

}  // namespace node
}  // namespace dai
