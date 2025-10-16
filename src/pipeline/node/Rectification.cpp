#include "depthai/pipeline/node/Rectification.hpp"

#include <chrono>

#include "depthai/pipeline/Pipeline.hpp"
#include "depthai/pipeline/datatype/BenchmarkReport.hpp"
#include "depthai/pipeline/datatype/MessageGroup.hpp"
#include "pipeline/ThreadedNodeImpl.hpp"
#include "utility/PimplImpl.hpp"

#if defined(DEPTHAI_HAVE_OPENCV_SUPPORT)
    #include <opencv2/calib3d.hpp>
#endif

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

}  // namespace

namespace dai {
namespace node {

void Rectification::setRunOnHost(bool runOnHost) {
    runOnHostVar = runOnHost;
}

bool Rectification::runOnHost() const {
    return runOnHostVar;
}

void Rectification::buildInternal() {
#if !defined(DEPTHAI_HAVE_OPENCV_SUPPORT)
    throw std::runtime_error("Rectification node requires OpenCV support!");
#endif
    auto platform = getParentPipeline().getDefaultDevice()->getPlatform();
    if(platform != Platform::RVC4 && runOnHost()) {
        throw std::runtime_error("Rectification node is only supported on RVC4 platform");
    }

    sync->out.link(inSync);
    sync->setRunOnHost(runOnHost());
}

#if !defined(DEPTHAI_HAVE_OPENCV_SUPPORT)
void Rectification::run() {
    throw std::runtime_error("Rectification node requires OpenCV support to run. Please enable OpenCV support in your build configuration.");
}
#else   // DEPTHAI_HAVE_OPENCV_SUPPORT

void Rectification::run() {
    auto& logger = pimpl->logger;
    using namespace std::chrono;

    auto start = steady_clock::now();

    bool initialized = false;
    cv::Mat map_x_l, map_y_l;
    cv::Mat map_x_r, map_y_r;
    while(isRunning()) {
        auto group = inSync.get<MessageGroup>();
        logger->error("Got message group");
        if(group == nullptr) continue;
        auto input1Frame = std::dynamic_pointer_cast<ImgFrame>(group->group.at(input1.getName()));
        auto input2Frame = std::dynamic_pointer_cast<ImgFrame>(group->group.at(input2.getName()));

        if(!initialized) {
            auto calib = getParentPipeline().getDefaultDevice()->getCalibration();

            auto leftSocket = (dai::CameraBoardSocket)input1Frame->getInstanceNum();
            auto rightSocket = (dai::CameraBoardSocket)input2Frame->getInstanceNum();
            auto inWarpWidth = input1Frame->getWidth();
            auto inWarpHeight = input1Frame->getHeight();
            auto outFrameWidth = inWarpWidth;
            auto outFrameHeight = inWarpHeight;

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
            cv::Size imageSize = cv::Size(inWarpWidth, inWarpHeight);
            std::tie(cv_R1, cv_R2) = computeRectificationMatrices(cv_M1, cv_d1, cv_M2, cv_d2, imageSize, cv_R, cv_T);

            auto cv_meshSize = cv::Size(outFrameWidth, outFrameHeight);

            auto cv_targetCamMatrix = cv_M2.clone();

            cv::initUndistortRectifyMap(cv_M1, cv_d1, cv_R1, cv_targetCamMatrix, cv_meshSize, CV_32FC1, map_x_l, map_y_l);
            cv::initUndistortRectifyMap(cv_M2, cv_d2, cv_R2, cv_targetCamMatrix, cv_meshSize, CV_32FC1, map_x_r, map_y_r);
            initialized = true;
        }
        logger->error("Got input frames");

        std::shared_ptr<dai::ImgFrame> rectifiedLeftFrame = std::make_shared<dai::ImgFrame>();
        std::shared_ptr<dai::ImgFrame> rectifiedRightFrame = std::make_shared<dai::ImgFrame>();
        size_t frameSize = input1Frame->getWidth() * input1Frame->getHeight();
        rectifiedLeftFrame->setData(std::vector<uint8_t>(frameSize));
        rectifiedRightFrame->setData(std::vector<uint8_t>(frameSize));

        rectifiedLeftFrame->setMetadata(*input1Frame);
        rectifiedLeftFrame->setWidth(input1Frame->getWidth());
        rectifiedLeftFrame->setHeight(input1Frame->getHeight());
        rectifiedLeftFrame->setType(dai::ImgFrame::Type::RAW8);
        rectifiedLeftFrame->fb.stride = rectifiedLeftFrame->fb.width * rectifiedLeftFrame->getBytesPerPixel();

        rectifiedRightFrame->setMetadata(*input2Frame);
        rectifiedRightFrame->setWidth(input2Frame->getWidth());
        rectifiedRightFrame->setHeight(input2Frame->getHeight());
        rectifiedRightFrame->setType(dai::ImgFrame::Type::RAW8);
        rectifiedRightFrame->fb.stride = rectifiedRightFrame->fb.width * rectifiedRightFrame->getBytesPerPixel();

        cv::Mat cv_input1 = input1Frame->getCvFrame();
        cv::Mat cv_input2 = input2Frame->getCvFrame();

        // convert to gray if needed
        if(cv_input1.channels() == 3) {
            cv::cvtColor(cv_input1, cv_input1, cv::COLOR_BGR2GRAY);
        }
        if(cv_input2.channels() == 3) {
            cv::cvtColor(cv_input2, cv_input2, cv::COLOR_BGR2GRAY);
        }

        cv::remap(cv_input1, rectifiedLeftFrame->getFrame(), map_x_l, map_y_l, cv::INTER_LINEAR, cv::BORDER_CONSTANT, cv::Scalar(0, 0, 0));
        cv::remap(cv_input2, rectifiedRightFrame->getFrame(), map_x_r, map_y_r, cv::INTER_LINEAR, cv::BORDER_CONSTANT, cv::Scalar(0, 0, 0));

        output1.send(rectifiedLeftFrame);
        output2.send(rectifiedRightFrame);
        logger->error("Sent output frames");

        // Passthrough the message
        passthrough1.send(input1Frame);
        passthrough2.send(input2Frame);
    }
}
#endif  // DEPTHAI_HAVE_OPENCV_SUPPORT

}  // namespace node
}  // namespace dai
