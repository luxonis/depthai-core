#include <basalt/serialization/headers_serialization.h>
#include <basalt/spline/se3_spline.h>

#include <rerun.hpp>
#define SOPHUS_USE_BASIC_LOGGING
#include <tbb/concurrent_queue.h>
#include <tbb/global_control.h>

#include "basalt/calibration/calibration.hpp"
#include "basalt/utils/vio_config.h"
#include "basalt/vi_estimator/vio_estimator.h"
#include "depthai/depthai.hpp"
#include "depthai/pipeline/datatype/TransformData.hpp"
#include "depthai/pipeline/ThreadedHostNode.hpp"

class BasaltVIO : public dai::NodeCRTP<dai::node::ThreadedHostNode, BasaltVIO> {
   public:
    constexpr static const char* NAME = "BasaltVIO";

    void stereoCB(std::shared_ptr<dai::ADatatype> images) {
        auto group = std::dynamic_pointer_cast<dai::MessageGroup>(images);
        int i = 0;
        basalt::OpticalFlowInput::Ptr data(new basalt::OpticalFlowInput(2));
        for(auto& msg : *group) {
            std::shared_ptr<dai::ImgFrame> imgFrame = std::dynamic_pointer_cast<dai::ImgFrame>(msg.second);
            if(i == 0) {
                leftImg = imgFrame;
            };
            auto t = imgFrame->getTimestamp();
            int64_t t_ns = std::chrono::time_point_cast<std::chrono::nanoseconds>(t).time_since_epoch().count();
            auto exposure = imgFrame->getExposureTime();

            int exposure_ms = std::chrono::duration_cast<std::chrono::milliseconds>(exposure).count();
            data->img_data[i].img.reset(new basalt::ManagedImage<uint16_t>(imgFrame->getWidth(), imgFrame->getHeight()));
            data->t_ns = t_ns;
            data->img_data[i].exposure = exposure_ms;
            size_t full_size = imgFrame->getWidth() * imgFrame->getHeight();
            const uint8_t* data_in = imgFrame->getData().data();
            uint16_t* data_out = data->img_data[i].img->ptr;
            for(size_t j = 0; j < full_size; j++) {
                int val = data_in[j];
                val = val << 8;
                data_out[j] = val;
            }
            i++;
        }
        last_img_data = data;
        if(image_data_queue) {
            image_data_queue->push(data);
        }
    };

    void imuCB(std::shared_ptr<dai::ADatatype> imu_data) {
        auto imuPackets = std::dynamic_pointer_cast<dai::IMUData>(imu_data);

        for(auto& imuPacket : imuPackets->packets) {
            basalt::ImuData<double>::Ptr data;
            data.reset(new basalt::ImuData<double>);
            auto t = imuPacket.acceleroMeter.getTimestamp();
            int64_t t_ns = std::chrono::time_point_cast<std::chrono::nanoseconds>(t).time_since_epoch().count();

            data->t_ns = t_ns;
            data->accel = Eigen::Vector3d(imuPacket.acceleroMeter.x, imuPacket.acceleroMeter.y, imuPacket.acceleroMeter.z);
            data->gyro = Eigen::Vector3d(imuPacket.gyroscope.x, imuPacket.gyroscope.y, imuPacket.gyroscope.z);
            if(imu_data_queue) imu_data_queue->push(data);
        }
    };

    void build() {

        vio_config.load("/workspaces/depthai_core_ws/basalt/data/msdmi_config.json");
        std::cout << vio_config.optical_flow_type << std::endl;
        inputImu.setMaxSize(0);
        inputStereo.setMaxSize(0);
        inputImu.setBlocking(false);
        inputStereo.setBlocking(false);
        inputImu.addCallback(std::bind(&BasaltVIO::imuCB, this, std::placeholders::_1));
        inputStereo.addCallback(std::bind(&BasaltVIO::stereoCB, this, std::placeholders::_1));
    }

    /**
     * Input for any ImgFrame messages
     * Default queue is blocking with size 8
     */
    Input inputStereo{*this, {.name="inStereo", .types={{dai::DatatypeEnum::MessageGroup, true}}}};
    Input inputImu{*this, {.name="inIMU", .types={{dai::DatatypeEnum::IMUData, true}}}};
    Output transform{*this, {.name="transform", .types={{dai::DatatypeEnum::TransformData, true}}}};
    Output passthrough{*this, {.name="imgPassthrough", .types={{dai::DatatypeEnum::ImgFrame, true}}}};

    void run() override {
        if(!calibrated) {
            calib = *exportCalibration();
            std::cout << "Calibration: " << calib.T_i_c[0].translation().transpose() << std::endl;

            opt_flow_ptr = basalt::OpticalFlowFactory::getOpticalFlow(vio_config, calib);
            std::cout << "Starting " << std::endl;
            opt_flow_ptr->start();
            image_data_queue = &opt_flow_ptr->input_img_queue;
            vio = basalt::VioEstimatorFactory::getVioEstimator(vio_config, calib, basalt::constants::g, true, true);
            vio->initialize(Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero());
            imu_data_queue = &vio->imu_data_queue;
            opt_flow_ptr->output_queue = &vio->vision_data_queue;
            vio->out_state_queue = &out_state_queue;
            vio->opt_flow_depth_guess_queue = &opt_flow_ptr->input_depth_queue;
            vio->opt_flow_state_queue = &opt_flow_ptr->input_state_queue;
            calibrated = true;
        }
        basalt::PoseVelBiasState<double>::Ptr data;
        while(isRunning()) {
            out_state_queue.pop(data);

            if(!data.get()) continue;
            auto final_pose = (data->T_w_i * calib.T_i_c[0]);
            auto trans = final_pose.translation();
            auto rot = final_pose.unit_quaternion();
            auto out = std::make_shared<dai::TransformData>(trans.x(), trans.y(), trans.z(), rot.x(), rot.y(), rot.z(), rot.w());

            double x, y, z;
            double roll, pitch, yaw;
            out->getTranslation(x, y, z);
            out->getRotationEuler(roll, pitch, yaw);
            transform.send(out);
            passthrough.send(leftImg);
        }
    }
    basalt::VioConfig vio_config;
    std::shared_ptr<basalt::Calibration<double>> calibInt;
    basalt::Calibration<double> calib;

    basalt::OpticalFlowBase::Ptr opt_flow_ptr;
    basalt::VioEstimatorBase::Ptr vio;
    basalt::OpticalFlowInput::Ptr last_img_data;

    tbb::concurrent_bounded_queue<basalt::OpticalFlowInput::Ptr>* image_data_queue;
    tbb::concurrent_bounded_queue<basalt::ImuData<double>::Ptr>* imu_data_queue;
    tbb::concurrent_bounded_queue<basalt::PoseVelBiasState<double>::Ptr> out_state_queue;

    std::vector<int64_t> vio_t_ns;
    Eigen::aligned_vector<Eigen::Vector3d> vio_t_w_i;
    std::shared_ptr<dai::ImgFrame> leftImg;
    bool calibrated = false;

    std::shared_ptr<basalt::Calibration<double>> exportCalibration() {
        auto pipeline = getParentPipeline();
        using Scalar = double;

        // if(calibInt.get()) return calibInt;

        calibInt.reset(new basalt::Calibration<Scalar>);
        calibInt->imu_update_rate = 200;

        // get camera ex-/intrinsics
        auto calibHandler = pipeline.getCalibrationData();

        // update after extrinsics are available
        // auto imuLeftExtrinsics = calibHandler.getCameraToImuExtrinsics(dai::CameraBoardSocket::LEFT);
        // auto imuRightExtrinsics = calibHandler.getCameraToImuExtrinsics(dai::CameraBoardSocket::RIGHT);
        // std vector of std vectors to Eigen::Matrix

        // Eigen::Matrix<Scalar, 4, 4> imuLeftExtrinsicsMatrix;
        // Eigen::Matrix<Scalar, 4, 4> imuRightExtrinsicsMatrix;

        // for (int i = 0; i < 4; i++) {
        //   for (int j = 0; j < 4; j++) {
        //     imuLeftExtrinsicsMatrix(i, j) = imuLeftExtrinsics[i][j];
        //     imuRightExtrinsicsMatrix(i, j) = imuRightExtrinsics[i][j];
        //   }
        // }
        // Eigen::Matrix3d rot = Eigen::Map<Eigen::Matrix3f>(ex.rotation);

        // For OAK D-PRO
        double roll = -3.1415;
        double pitch = 0.0;
        double yaw = -1.5708;
        Eigen::AngleAxisd roll_angle(roll, Eigen::Vector3d::UnitX());
        Eigen::AngleAxisd pitch_angle(pitch, Eigen::Vector3d::UnitY());
        Eigen::AngleAxisd yaw_angle(yaw, Eigen::Vector3d::UnitZ());

        Eigen::Quaterniond qL = roll_angle * pitch_angle * yaw_angle;

        Eigen::Quaterniond qR = roll_angle * pitch_angle * yaw_angle;
        // OAK D PRO
        Eigen::Vector3d transL(0.0, -0.06935, -0.00565);  // y x z for some reason
        Eigen::Vector3d transR(0.0, 0.0, -0.00565);

        // OAK D PRO W
        // Eigen::Vector3d transL(0.0, -0.075448, -0.0048);  // y x z for some reason
        // Eigen::Vector3d transR(0.0,0.0, -0.0048);

        basalt::Calibration<Scalar>::SE3 T_i_c_left(qL, transL);
        basalt::Calibration<Scalar>::SE3 T_i_c_right(qR, transR);

        calibInt->T_i_c.push_back(T_i_c_left);
        calibInt->T_i_c.push_back(T_i_c_right);

        // get resolution

        Eigen::Vector2i resolution;
        resolution << 640, 400;
        calibInt->resolution.push_back(resolution);

        auto leftIntrinsics = calibHandler.getCameraIntrinsics(dai::CameraBoardSocket::LEFT, 640, 400);
        auto rightIntrinsics = calibHandler.getCameraIntrinsics(dai::CameraBoardSocket::RIGHT, 640, 400);

        basalt::GenericCamera<Scalar> cameraL;
        basalt::PinholeCamera<Scalar>::VecN paramsL;
        // fx, fy, cx, cy
        double fxL = leftIntrinsics[0][0];
        double fyL = leftIntrinsics[1][1];
        double cxL = leftIntrinsics[0][2];
        double cyL = leftIntrinsics[1][2];
        paramsL << fxL, fyL, cxL, cyL;
        std::cout << "fxL: " << fxL << " fyL: " << fyL << " cxL: " << cxL << " cyL: " << cyL << std::endl;

        basalt::PinholeCamera<Scalar> pinholeL(paramsL);
        cameraL.variant = pinholeL;

        calibInt->intrinsics.push_back(cameraL);

        basalt::GenericCamera<Scalar> cameraR;
        basalt::PinholeCamera<Scalar>::VecN paramsR;
        // fx, fy, cx, cy
        double fxR = rightIntrinsics[0][0];
        double fyR = rightIntrinsics[1][1];
        double cxR = rightIntrinsics[0][2];
        double cyR = rightIntrinsics[1][2];
        paramsR << fxR, fyR, cxR, cyR;

        basalt::PinholeCamera<Scalar> pinholeR(paramsR);
        cameraR.variant = pinholeR;

        calibInt->intrinsics.push_back(cameraR);

        return calibInt;
    }
};

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
    Input inputTrans{*this, {.name="inTrans",.types={{dai::DatatypeEnum::TransformData, true}}}};
    Input inputImg{*this, {.name="inImg",.types={{dai::DatatypeEnum::ImgFrame, true}}}};
    void run() override {
        const auto rec = rerun::RecordingStream("rerun");
        rec.spawn().exit_on_failure();
        rec.log_timeless("world", rerun::ViewCoordinates::RDF);
        rec.log("world/ground", rerun::Boxes3D::from_half_sizes({{3.f, 3.f, 0.00001f}}));

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
                rec.log("world/camera/image", rerun::Pinhole::from_focal_length_and_resolution({398.554f, 398.554f}, {640.0f, 400.0f}));
                rec.log("world/camera/image/rgb",
                        rerun::Image(tensor_shape(imgFrame->getCvFrame()), reinterpret_cast<const uint8_t*>(imgFrame->getCvFrame().data)));
            }
        }
    }
    std::vector<rerun::Vec3D> positions;
};

int main() {
    using namespace std;
    std::unique_ptr<tbb::global_control> tbb_global_control;

    // Create pipeline
    dai::Pipeline pipeline;
    int fps = 60;
    int width = 640;
    int height = 400;
    // Define sources and outputs
    auto left = pipeline.create<dai::node::MonoCamera>();
    auto right = pipeline.create<dai::node::MonoCamera>();
    auto stereo = pipeline.create<dai::node::StereoDepth>();
    auto imu = pipeline.create<dai::node::IMU>();
    auto sync = pipeline.create<dai::node::Sync>();
    auto odom = pipeline.create<BasaltVIO>();
    auto rerun = pipeline.create<RerunStreamer>();
    auto xoutDepth = pipeline.create<dai::node::XLinkOut>();
    xoutDepth->setStreamName("depth");
    imu->enableIMUSensor({dai::IMUSensor::ACCELEROMETER_RAW, dai::IMUSensor::GYROSCOPE_RAW}, 200);
    imu->setBatchReportThreshold(1);
    imu->setMaxBatchReports(10);

    left->setCamera("left");
    left->setResolution(dai::MonoCameraProperties::SensorResolution::THE_400_P);
    left->setFps(fps);
    right->setCamera("right");
    right->setResolution(dai::MonoCameraProperties::SensorResolution::THE_400_P);
    right->setFps(fps);

    // Linking
    left->out.link(stereo->left);
    right->out.link(stereo->right);
    stereo->rectifiedLeft.link(sync->inputs["left"]);
    stereo->rectifiedRight.link(sync->inputs["right"]);
    stereo->depth.link(xoutDepth->input);
    sync->out.link(odom->inputStereo);
    imu->out.link(odom->inputImu);
    odom->transform.link(rerun->inputTrans);
    odom->passthrough.link(rerun->inputImg);

    pipeline.start();
    pipeline.wait();
}
