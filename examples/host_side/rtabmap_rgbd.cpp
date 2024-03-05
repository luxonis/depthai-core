#include <iostream>

// Includes common necessary includes for development using depthai library
#include "depthai/depthai.hpp"
#include "depthai/pipeline/node/test/MyConsumer.hpp"
#include "depthai/pipeline/node/test/MyProducer.hpp"
#include "rtabmap/core/CameraModel.h"
#include "rtabmap/core/Odometry.h"
#include "rtabmap/core/OdometryInfo.h"
#include "rtabmap/core/Rtabmap.h"
#include "rtabmap/core/SensorData.h"
#include "rtabmap/core/Transform.h"

// shared
#include "depthai/properties/XLinkOutProperties.hpp"

// project
#include "depthai/pipeline/ThreadedNode.hpp"
#include "depthai/pipeline/datatype/Buffer.hpp"
#include "depthai/pipeline/datatype/ImgFrame.hpp"
// Optional. If set (true), the ColorCamera is downscaled from 1080p to 720p.
// Otherwise (false), the aligned depth is automatically upscaled to 1080p
static std::atomic<bool> downscaleColor{true};
static constexpr int fps = 30;
// The disparity is computed at this resolution, then upscaled to RGB resolution
static constexpr auto monoRes = dai::MonoCameraProperties::SensorResolution::THE_720_P;

static float rgbWeight = 0.4f;
static float depthWeight = 0.6f;

static void updateBlendWeights(int percentRgb, void* ctx) {
    rgbWeight = float(percentRgb) / 100.f;
    depthWeight = 1.f - rgbWeight;
}
// class RTABMAPOdom : public dai::NodeCRTP<dai::ThreadedNode, RTABMAPOdom> {
//    public:
//     constexpr static const char* NAME = "RTABMAPOdom";

//    public:
//     void build() {
//         hostNode = true;
//     }

//     /**
//      * Input for any ImgFrame messages to be displayed
//      * Default queue is blocking with size 8
//      */
//     Input input{true, *this, "in", Input::Type::SReceiver, true, 8, true, {{dai::DatatypeEnum::Buffer, true}}};

//     void run() override {
//         while(isRunning()) {
//             std::shared_ptr<dai::ImgFrame> imgFrame = input.queue.get<dai::ImgFrame>();
//             if(imgFrame != nullptr) {
//                 cv::imshow("MyConsumer", imgFrame->getCvFrame());

//                 auto key = cv::waitKey(1);
//                 if(key == 'q') {
//                     stop();
//                 }            }
//         }
//         fmt::print("Display node stopped\n");
//     }
// };

int main() {
    using namespace std;

    int width = 1280;
    int height = 720;
    // Create pipeline
    dai::Pipeline pipeline;
    dai::Device device;
    std::vector<std::string> queueNames;

    // Define sources and outputs
    auto camRgb = pipeline.create<dai::node::ColorCamera>();
    auto left = pipeline.create<dai::node::MonoCamera>();
    auto right = pipeline.create<dai::node::MonoCamera>();
    auto stereo = pipeline.create<dai::node::StereoDepth>();
    // auto imu = pipeline.create<dai::node::IMU>();
    // auto featureTracker = pipeline.create<dai::node::FeatureTracker>();

    auto rgbOut = pipeline.create<dai::node::XLinkOut>();
    auto depthOut = pipeline.create<dai::node::XLinkOut>();
    // auto imuOut = pipeline.create<dai::node::XLinkOut>();
    // auto featureTrackerOut = pipeline.create<dai::node::XLinkOut>();

    rgbOut->setStreamName("rgb");
    depthOut->setStreamName("depth");
    // imuOut->setStreamName("imu");
    // featureTrackerOut->setStreamName("featureTracker");
    // imu->enableIMUSensor({dai::IMUSensor::ACCELEROMETER_RAW, dai::IMUSensor::GYROSCOPE_RAW}, 100);
    // imu->setBatchReportThreshold(1);
    // imu->out.link(imuOut->input);
    // camRgb->video.link(featureTracker->inputImage);


    camRgb->setBoardSocket(dai::CameraBoardSocket::CAM_A);
    camRgb->setResolution(dai::ColorCameraProperties::SensorResolution::THE_1080_P);
    camRgb->setFps(fps);
    if(downscaleColor) camRgb->setIspScale(2, 3);

    try {
        auto calibData = device.readCalibration2();
        auto lensPosition = calibData.getLensPosition(dai::CameraBoardSocket::CAM_A);
        if(lensPosition) {
            camRgb->initialControl.setManualFocus(lensPosition);
        }
    } catch(const std::exception& ex) {
        std::cout << ex.what() << std::endl;
        return 1;
    }

    left->setResolution(monoRes);
    left->setCamera("left");
    left->setFps(fps);
    right->setResolution(monoRes);
    right->setCamera("right");
    right->setFps(fps);

    stereo->setDefaultProfilePreset(dai::node::StereoDepth::PresetMode::HIGH_DENSITY);
    // LR-check is required for depth alignment
    stereo->setLeftRightCheck(true);
    stereo->setDepthAlign(dai::CameraBoardSocket::CAM_A);

    // Linking
    camRgb->isp.link(rgbOut->input);
    left->out.link(stereo->left);
    right->out.link(stereo->right);
    stereo->depth.link(depthOut->input);

    // Connect to device and start pipeline
    device.startPipeline(pipeline);

    dai::CalibrationHandler calibHandler = device.readCalibration();

    auto cameraId = dai::CameraBoardSocket::CAM_A;
    cv::Mat cameraMatrix, distCoeffs, newCameraMatrix;

    std::vector<std::vector<float> > matrix = calibHandler.getCameraIntrinsics(cameraId, width, height);
    cameraMatrix = (cv::Mat_<double>(3, 3) << matrix[0][0],
                    matrix[0][1],
                    matrix[0][2],
                    matrix[1][0],
                    matrix[1][1],
                    matrix[1][2],
                    matrix[2][0],
                    matrix[2][1],
                    matrix[2][2]);

    std::vector<float> coeffs = calibHandler.getDistortionCoefficients(cameraId);
    if(calibHandler.getDistortionModel(cameraId) == dai::CameraModel::Perspective)
        distCoeffs = (cv::Mat_<double>(1, 8) << coeffs[0], coeffs[1], coeffs[2], coeffs[3], coeffs[4], coeffs[5], coeffs[6], coeffs[7]);

    newCameraMatrix = cameraMatrix;

    double fx = newCameraMatrix.at<double>(0, 0);
    double fy = newCameraMatrix.at<double>(1, 1);
    double cx = newCameraMatrix.at<double>(0, 2);
    double cy = newCameraMatrix.at<double>(1, 2);
    double baseline = calibHandler.getBaselineDistance(dai::CameraBoardSocket::CAM_C, dai::CameraBoardSocket::CAM_B) / 100.0;

    auto model = rtabmap::CameraModel(device.getDeviceName(), fx, fy, cx, cy, rtabmap::Transform::getIdentity(), baseline, cv::Size(width, height));

    auto rgbQ = device.getOutputQueue("rgb", 8, false);
    auto depthQ = device.getOutputQueue("depth", 8, false);
    // auto imuQ = device.getOutputQueue("imu", 8, false);
    // auto featureTrackerQ = device.getOutputQueue("featureTracker", 8, false);
    int seq = 0;
    auto odom = rtabmap::Odometry::create();
    while(true) {
        auto rgbFrame = rgbQ->get<dai::ImgFrame>();
        auto depthFrame = depthQ->get<dai::ImgFrame>();
        // auto imuData = imuQ->get<dai::IMUData>();
        double stamp = std::chrono::duration<double>(rgbFrame->getTimestampDevice(dai::CameraExposureOffset::MIDDLE).time_since_epoch()).count();
        auto data = rtabmap::SensorData(rgbFrame->getCvFrame(), depthFrame->getCvFrame(), model, seq++, stamp);

        rtabmap::OdometryInfo info;
        auto pose = odom->process(data, &info);
        std::cout << pose << std::endl;
    }
    return 0;
}
