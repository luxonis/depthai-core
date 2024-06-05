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
#include "depthai/rtabmap/RTABMapSLAM.hpp"
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
    Input inputPCL{*this, {.name="inPCL", .types={{dai::DatatypeEnum::PointCloudData, true}}}};
    Input inputMap{*this, {.name="inMap", .types={{dai::DatatypeEnum::ImgFrame, true}}}};

    void run() override {
        const auto rec = rerun::RecordingStream("rerun");
        rec.spawn().exit_on_failure();
        rec.log_timeless("world", rerun::ViewCoordinates::FLU);
        rec.log("world/ground", rerun::Boxes3D::from_half_sizes({{3.f, 3.f, 0.00001f}}));

        while(isRunning()) {
            std::shared_ptr<dai::TransformData> transData = inputTrans.get<dai::TransformData>();
            auto imgFrame = inputImg.get<dai::ImgFrame>();
            auto pclData = inputPCL.tryGet<dai::PointCloudData>();
            auto mapData = inputMap.tryGet<dai::ImgFrame>();
            if(transData != nullptr) {
                double x, y, z, qx, qy, qz, qw;
                transData->getTranslation(x, y, z);
                transData->getQuaternion(qx, qy, qz, qw);

                // //write matrix data to file, one matrix per line, values separated by commas
                // std::vector<std::vector<double>> data = transData->transform.data;
                // std::ofstream file;
                // file.open("/workspaces/depthai_core_ws/positions.csv", std::ios::app);
                // for (int i = 0; i < data.size(); i++) {
                //     for (int j = 0; j < data[i].size(); j++) {
                //         file << data[i][j] << ",";
                //     }
                // }
                // file << "\n";
                
                auto position = rerun::Vec3D(x, y, z);

                rec.log("world/camera", rerun::Transform3D(position, rerun::datatypes::Quaternion::from_xyzw(qx, qy, qz, qw)));
                positions.push_back(position);
                rerun::LineStrip3D lineStrip(positions);
                rec.log("world/trajectory", rerun::LineStrips3D(lineStrip));
                rec.log("world/camera/image", rerun::Pinhole::from_focal_length_and_resolution({398.554f, 398.554f}, {640.0f, 400.0f}).with_camera_xyz(rerun::components::ViewCoordinates::FLU));
                rec.log("world/camera/image/rgb",
                        rerun::Image(tensor_shape(imgFrame->getCvFrame()), reinterpret_cast<const uint8_t*>(imgFrame->getCvFrame().data)));
                if(pclData != nullptr) {
                    std::vector<rerun::Position3D> points;
                    for(auto& point : pclData->points) {
                        points.push_back(rerun::Position3D(point.x, point.y, point.z));
                    }
                    rec.log("world/camera/pointcloud", rerun::Points3D(points));
                }
                if(mapData != nullptr) {
                    rec.log("map", rerun::Image(tensor_shape(mapData->getCvFrame()), reinterpret_cast<const uint8_t*>(mapData->getCvFrame().data)));
                }
            }
        }
    }
    std::vector<rerun::Vec3D> positions;
};

int main() {
    using namespace std;
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
    auto slam = pipeline.create<dai::node::RTABMapSLAM>()->build();
    auto rerun = pipeline.create<RerunStreamer>();
    auto params = rtabmap::ParametersMap();
    params.insert(rtabmap::ParametersPair(rtabmap::Parameters::kOdomResetCountdown(), "30"));

    odom->setParams(params);

    params.insert(rtabmap::ParametersPair(rtabmap::Parameters::kRGBDCreateOccupancyGrid(), "true"));
    slam->setParams(params);
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
    stereo->setSubpixel(true);
    stereo->setDefaultProfilePreset(dai::node::StereoDepth::PresetMode::HIGH_DENSITY);
    stereo->setLeftRightCheck(true);
    stereo->setRectifyEdgeFillColor(0); // black, to better see the cutout
	stereo->enableDistortionCorrection(true);
    stereo->initialConfig.setLeftRightCheckThreshold(10);
    stereo->setDepthAlign(dai::StereoDepthProperties::DepthAlign::RECTIFIED_LEFT);

    // Linking
    left->out.link(stereo->left);
    right->out.link(stereo->right);
    featureTracker->passthroughInputImage.link(odom->inputRect);
    stereo->depth.link(odom->inputDepth);
    imu->out.link(odom->inputIMU);
    featureTracker->outputFeatures.link(odom->inputFeatures);

    odom->transform.link(slam->inputOdomPose);
    odom->passthroughRect.link(slam->inputRect);
    odom->passthroughDepth.link(slam->inputDepth);
    //odom->passthroughFeatures.link(slam->inputFeatures);

    slam->transform.link(rerun->inputTrans);
    slam->passthroughRect.link(rerun->inputImg);
    slam->occupancyMap.link(rerun->inputMap);

    pipeline.start();
    pipeline.wait();
}
