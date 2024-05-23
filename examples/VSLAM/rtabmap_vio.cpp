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
#include <rerun.hpp>

#include "depthai/pipeline/ThreadedHostNode.hpp"
#include "depthai/pipeline/datatype/Buffer.hpp"
#include "depthai/pipeline/datatype/ImgFrame.hpp"
#include "depthai/pipeline/datatype/TransformData.hpp"
#include "depthai/rtabmap/RTABMapVIO.hpp"
rerun::Collection<rerun::TensorDimension> tensor_shape(const cv::Mat& img) {
    return {img.rows, img.cols, img.channels()};
};
class RerunStreamer : public dai::NodeCRTP<dai::node::ThreadedHostNode, RerunStreamer> {
   public:
    constexpr static const char* NAME = "RerunStreamer";

   public:
    void build() {
    }

    /**
     * Input for any ImgFrame messages 
     * Default queue is blocking with size 8
     */
    Input inputTrans{*this, {.name="inTrans", .types={{dai::DatatypeEnum::TransformData, true}}}};
    Input inputImg{*this, {.name="inImg", .types={{dai::DatatypeEnum::ImgFrame, true}}}};
    void run() override {
        const auto rec = rerun::RecordingStream("rerun");
        rec.spawn().exit_on_failure();
        rec.log_timeless("world", rerun::ViewCoordinates::RDF);

        while(isRunning()) {
            std::shared_ptr<dai::TransformData> transData = inputTrans.get<dai::TransformData>();
            auto imgFrame = inputImg.get<dai::ImgFrame>();
            if(transData != nullptr) {
                double x, y, z, qx, qy, qz, qw;
                transData->getTranslation(x, y, z);
                transData->getQuaternion(qx, qy, qz, qw);
                auto position = rerun::Vec3D(x, y, z);

                rec.log("world/camera", rerun::Transform3D(position, rerun::datatypes::Quaternion::from_xyzw(qx, qy, qz, qw)));
                positions.push_back(position);
                rerun::LineStrip3D lineStrip(positions);
                rec.log("world/trajectory", rerun::LineStrips3D(lineStrip));
                rec.log("world/camera/image", rerun::Pinhole::from_focal_length_and_resolution({256.06f, 256.06f}, {576.0f, 360.0f}));
                rec.log("world/camera/image/rgb",
                        rerun::Image(tensor_shape(imgFrame->getCvFrame()), reinterpret_cast<const uint8_t*>(imgFrame->getCvFrame().data)));
            }
        }
    }
    std::vector<rerun::Vec3D> positions;
};


int main() {
    using namespace std;
    // ULogger::setType(ULogger::kTypeConsole);
	// ULogger::setLevel(ULogger::kDebug);
    // Create pipeline
    dai::Pipeline pipeline;
    int fps = 30;
    int width = 640;
    int height = 400;
    // Define sources and outputs
    auto left = pipeline.create<dai::node::MonoCamera>()->build();
    auto right = pipeline.create<dai::node::MonoCamera>()->build();
    auto stereo = pipeline.create<dai::node::StereoDepth>();
    auto imu = pipeline.create<dai::node::IMU>()->build();
    auto featureTracker = pipeline.create<dai::node::FeatureTracker>()->build();
    auto odom = pipeline.create<dai::node::RTABMapVIO>()->build();
    auto rerun = pipeline.create<RerunStreamer>();
    auto params = rtabmap::ParametersMap();
    params.insert(rtabmap::ParametersPair(rtabmap::Parameters::kOdomResetCountdown(), "30"));

    odom->setParams(params);
    imu->enableIMUSensor({dai::IMUSensor::ACCELEROMETER_RAW, dai::IMUSensor::GYROSCOPE_RAW}, 100);
    imu->setBatchReportThreshold(1);
    imu->setMaxBatchReports(10);

    featureTracker->setHardwareResources(1, 2);
    featureTracker->initialConfig.setCornerDetector(dai::FeatureTrackerConfig::CornerDetector::Type::SHI_THOMASI);
    featureTracker->initialConfig.setNumTargetFeatures(1000);
    featureTracker->initialConfig.setMotionEstimator(false);
    featureTracker->initialConfig.featureMaintainer.minimumDistanceBetweenFeatures = 49.0;
    stereo->rectifiedLeft.link(featureTracker->inputImage);
    // stereo->setAlphaScaling(0.0);
    left->setResolution(dai::MonoCameraProperties::SensorResolution::THE_400_P);
    left->setCamera("left");
    left->setFps(fps);
    right->setResolution(dai::MonoCameraProperties::SensorResolution::THE_400_P);
    right->setCamera("right");
    right->setFps(fps);
	stereo->setExtendedDisparity(false);

    stereo->setDefaultProfilePreset(dai::node::StereoDepth::PresetMode::HIGH_DENSITY);
    stereo->setLeftRightCheck(true);
    stereo->setRectifyEdgeFillColor(0); // black, to better see the cutout
	stereo->enableDistortionCorrection(true);
    stereo->initialConfig.setLeftRightCheckThreshold(10);
    stereo->setDepthAlign(dai::StereoDepthProperties::DepthAlign::RECTIFIED_LEFT);


    // auto controlIn = pipeline.create<dai::node::XLinkIn>();
    // controlIn->setStreamName("control");

    // Linking
    left->out.link(stereo->left);
    right->out.link(stereo->right);
    featureTracker->passthroughInputImage.link(odom->inputRect);
    stereo->depth.link(odom->inputDepth);
    imu->out.link(odom->inputIMU);
    featureTracker->outputFeatures.link(odom->inputFeatures);
    odom->transform.link(rerun->inputTrans);
    odom->passthroughRect.link(rerun->inputImg);
    pipeline.start();
    pipeline.wait();
}
