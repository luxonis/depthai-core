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
static constexpr int fps = 60;
// The disparity is computed at this resolution, then upscaled to RGB resolution
static constexpr auto monoRes = dai::MonoCameraProperties::SensorResolution::THE_400_P;

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

    int width = 640;
    int height = 400;
    // Create pipeline
    dai::Pipeline pipeline;
    dai::Device device;
    std::vector<std::string> queueNames;

    // Define sources and outputs
    auto left = pipeline.create<dai::node::MonoCamera>();
    auto right = pipeline.create<dai::node::MonoCamera>();
    auto stereo = pipeline.create<dai::node::StereoDepth>();
    auto imu = pipeline.create<dai::node::IMU>();
    // auto featureTracker = pipeline.create<dai::node::FeatureTracker>();

    auto leftOut = pipeline.create<dai::node::XLinkOut>();
    auto depthOut = pipeline.create<dai::node::XLinkOut>();
    auto imuOut = pipeline.create<dai::node::XLinkOut>();
    // auto featureTrackerOut = pipeline.create<dai::node::XLinkOut>();

    leftOut->setStreamName("left");
    depthOut->setStreamName("depth");
    imuOut->setStreamName("imu");
    // featureTrackerOut->setStreamName("featureTracker");
    imu->enableIMUSensor({dai::IMUSensor::ACCELEROMETER_RAW, dai::IMUSensor::GYROSCOPE_RAW}, 100);
    imu->setBatchReportThreshold(1);
    imu->setMaxBatchReports(10);
    imu->out.link(imuOut->input);


    // featureTracker->setHardwareResources(1, 2);
    // featureTracker->initialConfig.setCornerDetector(dai::FeatureTrackerConfig::CornerDetector::Type::SHI_THOMASI);
    // featureTracker->initialConfig.setNumTargetFeatures(1000);
    // featureTracker->initialConfig.setMotionEstimator(false);
    // featureTracker->initialConfig.featureMaintainer.minimumDistanceBetweenFeatures = 50.0;
    // featureTracker->inputImage.setBlocking(false);
    // stereo->rectifiedLeft.link(featureTracker->inputImage);

    left->setResolution(monoRes);
    left->setCamera("left");
    left->setFps(fps);
    right->setResolution(monoRes);
    right->setCamera("right");
    right->setFps(fps);

    stereo->setDefaultProfilePreset(dai::node::StereoDepth::PresetMode::HIGH_DENSITY);
    // LR-check is required for depth alignment
    stereo->setLeftRightCheck(true);
    stereo->initialConfig.setLeftRightCheckThreshold(10);
    stereo->setDepthAlign(dai::CameraBoardSocket::CAM_B);

    // Linking
    left->out.link(stereo->left);
    right->out.link(stereo->right);
    stereo->rectifiedLeft.link(leftOut->input);
    leftOut->input.setBlocking(false);
    stereo->depth.link(depthOut->input);

    // Connect to device and start pipeline
    device.startPipeline(pipeline);

    dai::CalibrationHandler calibHandler = device.readCalibration();

    auto cameraId = dai::CameraBoardSocket::CAM_B;
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

    auto model = rtabmap::StereoCameraModel(device.getDeviceName(), fx, fy, cx, cy, baseline, rtabmap::Transform::getIdentity(), cv::Size(width, height));

    auto leftQ = device.getOutputQueue("left", 8, false);
    auto depthQ = device.getOutputQueue("depth", 8, false);
    auto imuQ = device.getOutputQueue("imu", 8, false);
    // auto featureTrackerQ = device.getOutputQueue("featureTracker", 8, false);
    int seq = 0;
    auto odom = rtabmap::Odometry::create();
    while(true) {
        auto leftFrame = leftQ->get<dai::ImgFrame>();
        auto depthFrame = depthQ->get<dai::ImgFrame>();
        // auto features = featureTrackerQ->get<dai::TrackedFeatures>();
        auto imuData = imuQ->get<dai::IMUData>();
        double stamp = std::chrono::duration<double>(leftFrame->getTimestampDevice(dai::CameraExposureOffset::MIDDLE).time_since_epoch()).count();
        auto data = rtabmap::SensorData(leftFrame->getCvFrame(), depthFrame->getCvFrame(), model.left(), seq++, stamp);
        std::vector<cv::KeyPoint> keypoints;

        // for (auto &feature : features->trackedFeatures) {
        //     keypoints.emplace_back(cv::KeyPoint(feature.position.x, feature.position.y, 3));
        // }
        rtabmap::OdometryInfo info;
        // data.setFeatures(keypoints, std::vector<cv::Point3f>(), cv::Mat());
        auto pose = odom->process(data, &info);
        cv::Mat final_img;

        for(auto word : info.words) {
            keypoints.push_back(word.second);
        }
        cv::drawKeypoints(leftFrame->getCvFrame(), keypoints, final_img);

        // add pose information to frame

        float x, y, z, roll, pitch, yaw;
        pose.getTranslationAndEulerAngles(x,y,z,roll,pitch,yaw);

        std::stringstream xPos;
        xPos << "X: " << x << " mm";
        cv::putText(final_img, xPos.str(), cv::Point(10, 50), cv::FONT_HERSHEY_TRIPLEX, 0.5, 255);

        std::stringstream yPos;
        yPos << "Y: " << y << " mm";
        cv::putText(final_img, yPos.str(), cv::Point(10, 70), cv::FONT_HERSHEY_TRIPLEX, 0.5, 255);

        std::stringstream zPos;
        zPos << "Z: " << z << " mm";
        cv::putText(final_img, zPos.str(), cv::Point(10, 90), cv::FONT_HERSHEY_TRIPLEX, 0.5, 255);

        std::stringstream rollPos;
        rollPos << "Roll: " << roll << " deg";
        cv::putText(final_img, rollPos.str(), cv::Point(10, 110), cv::FONT_HERSHEY_TRIPLEX, 0.5, 255);

        std::stringstream pitchPos;
        pitchPos << "Pitch: " << pitch << " deg";
        cv::putText(final_img, pitchPos.str(), cv::Point(10, 130), cv::FONT_HERSHEY_TRIPLEX, 0.5, 255);

        std::stringstream yawPos;
        yawPos << "Yaw: " << yaw << " deg";
        cv::putText(final_img, yawPos.str(), cv::Point(10, 150), cv::FONT_HERSHEY_TRIPLEX, 0.5, 255);

        cv::imshow("keypoints", final_img);
        auto key = cv::waitKey(1);
        if(key == 'q' || key == 'Q') {
            return 0;
        }
        std::cout << pose << std::endl;
    }
    return 0;
}
