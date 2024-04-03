// #include <basalt/serialization/headers_serialization.h>
// #include <basalt/spline/se3_spline.h>
// #include <basalt/utils/vis_utils.h>
#include <basalt/vi_estimator/vio_estimator.h>

#include <basalt/calibration/calibration.hpp>
#include <rerun.hpp>

#include "depthai/depthai.hpp"
#include "depthai/pipeline/datatype/TransformData.hpp"

class BasaltVIO : public dai::NodeCRTP<dai::ThreadedNode, BasaltVIO> {
   public:
    constexpr static const char* NAME = "BasaltVIO";

   public:
    void build() {
        hostNode = true;
        calib->imu_update_rate = 200;
        auto pipeline = getParentPipeline();
    auto device = pipeline.getDevice();
    auto calibHandler = device->readCalibration2();

        auto imuExtr = calibHandler.getCameraToImuExtrinsics(dai::CameraBoardSocket::LEFT,true);
        dai::TransformData transData(imuExtr);
        float x,y,z,qx,qy,qz,qw;
        transData.getTranslation(x,y,z);
        transData.getQuaternion(qx,qy,qz,qw);
        Eigent::Vector3f t(x,y,z);
        Eigen::Quaterniond q(qw,qx,qy,qz);
        basalt::Calibration<double>::SE3 T_i_c(q,t);
        
        std::vector<std::vector<float> > matrix = getCameraIntrinsics(dai::CameraBoardSocket::LEFT);
        Eigen::Vector2i resolution;
        resolution << 640, 480;
        calib->resolution.push_back(resolution);
        basalt::GenericCamera<double> camera;
        basalt::PinholeCamera<double>::VecN params;
        params << matrix[0][0], matrix[1][1], matrix[0][2], matrix[1][2];
        basalt::PinholeCamera pinhole(params);
        camera.variant = pinhole;
        calib->intrinsics.push_back(camera);
    }

    /**
     * Input for any ImgFrame messages
     * Default queue is blocking with size 8
     */
    Input inputStereo{true, *this, "inStereo", Input::Type::SReceiver, false, 8, true, {{dai::DatatypeEnum::MessageGroup, true}}};
    Input inputImu{true, *this, "inIMU", Input::Type::SReceiver, false, 8, true, {{dai::DatatypeEnum::IMUData, true}}};
    void run() override {
        while(isRunning()) {
        
        }
    }
    basalt::VioConfig config;
    basalt::Calibration<double> calib;
    basalt::VioEstimatorBase::Ptr vio;
    basalt::OpticalFlowBase::Ptr optical_flow;
};

rerun::Collection<rerun::TensorDimension> tensor_shape(const cv::Mat& img) {
    return {img.rows, img.cols, img.channels()};
};
class RerunStreamer : public dai::NodeCRTP<dai::ThreadedNode, RerunStreamer> {
   public:
    constexpr static const char* NAME = "RerunStreamer";

   public:
    void build() {
        hostNode = true;
    }

    /**
     * Input for any ImgFrame messages
     * Default queue is blocking with size 8
     */
    Input inputTrans{true, *this, "inTrans", Input::Type::SReceiver, false, 8, true, {{dai::DatatypeEnum::TransformData, true}}};
    Input inputImg{true, *this, "inImg", Input::Type::SReceiver, false, 8, true, {{dai::DatatypeEnum::ImgFrame, true}}};
    void run() override {
        const auto rec = rerun::RecordingStream("rerun");
        rec.spawn().exit_on_failure();
        rec.log_timeless("world", rerun::ViewCoordinates::RDF);

        while(isRunning()) {
            std::shared_ptr<dai::TransformData> transData = inputTrans.queue.get<dai::TransformData>();
            auto imgFrame = inputImg.queue.get<dai::ImgFrame>();
            if(transData != nullptr) {
                float x, y, z, qx, qy, qz, qw;
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
    auto left = pipeline.create<dai::node::ColorCamera>();
    auto right = pipeline.create<dai::node::ColorCamera>();
    auto stereo = pipeline.create<dai::node::StereoDepth>();
    auto imu = pipeline.create<dai::node::IMU>();
    auto featureTracker = pipeline.create<dai::node::FeatureTracker>();
    auto odom = pipeline.create<dai::node::RTABMapVIO>();
    auto rerun = pipeline.create<RerunStreamer>();
    auto params = rtabmap::ParametersMap();
    params.insert(rtabmap::ParametersPair(rtabmap::Parameters::kOdomResetCountdown(), "30"));

    odom->setParams(params);
    imu->enableIMUSensor({dai::IMUSensor::ACCELEROMETER_RAW, dai::IMUSensor::GYROSCOPE_RAW}, 100);
    imu->enableIMUSensor({dai::IMUSensor::ROTATION_VECTOR}, 100);
    imu->setBatchReportThreshold(1);
    imu->setMaxBatchReports(10);

    featureTracker->setHardwareResources(1, 2);
    featureTracker->initialConfig.setCornerDetector(dai::FeatureTrackerConfig::CornerDetector::Type::SHI_THOMASI);
    featureTracker->initialConfig.setNumTargetFeatures(1000);
    featureTracker->initialConfig.setMotionEstimator(false);
    featureTracker->initialConfig.featureMaintainer.minimumDistanceBetweenFeatures = 49.0;
    stereo->rectifiedLeft.link(featureTracker->inputImage);
    stereo->setAlphaScaling(0.0);
    left->setIspScale(9, 20);
    left->setCamera("left");
    left->setFps(fps);
    right->setIspScale(9, 20);
    right->setCamera("right");
    right->setFps(fps);
    stereo->setExtendedDisparity(false);

    stereo->setDefaultProfilePreset(dai::node::StereoDepth::PresetMode::HIGH_DENSITY);
    stereo->setLeftRightCheck(true);
    stereo->setRectifyEdgeFillColor(0);  // black, to better see the cutout
    stereo->enableDistortionCorrection(true);
    stereo->initialConfig.setLeftRightCheckThreshold(10);
    stereo->setDepthAlign(dai::StereoDepthProperties::DepthAlign::RECTIFIED_LEFT);

    // auto controlIn = pipeline.create<dai::node::XLinkIn>();
    // controlIn->setStreamName("control");

    // Linking
    left->isp.link(stereo->left);
    right->isp.link(stereo->right);
    featureTracker->passthroughInputImage.link(odom->inputRect);
    stereo->depth.link(odom->inputDepth);
    imu->out.link(odom->inputIMU);
    featureTracker->outputFeatures.link(odom->inputFeatures);
    odom->transform.link(rerun->inputTrans);
    odom->passthroughRect.link(rerun->inputImg);
    pipeline.start();
    pipeline.wait();
}
