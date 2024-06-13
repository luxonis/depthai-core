#include "depthai/basalt/BasaltVIO.hpp"

#include "depthai/pipeline/Pipeline.hpp"
#include "depthai/pipeline/datatype/MessageGroup.hpp"
#include "tbb/concurrent_queue.h"
#include "tbb/global_control.h"
#include "../utility/PimplImpl.hpp"
namespace dai {


namespace node {

class BasaltVIO::Impl {
   public:
    Impl() = default;
    std::shared_ptr<tbb::concurrent_bounded_queue<basalt::OpticalFlowInput::Ptr>> imageDataQueue;
    std::shared_ptr<tbb::concurrent_bounded_queue<basalt::ImuData<double>::Ptr>> imuDataQueue;
    std::shared_ptr<tbb::concurrent_bounded_queue<basalt::PoseVelBiasState<double>::Ptr>> outStateQueue;
    std::shared_ptr<tbb::detail::d1::global_control> tbbGlobalControl;
};

BasaltVIO::BasaltVIO(){}
BasaltVIO::~BasaltVIO() = default;

std::shared_ptr<BasaltVIO> BasaltVIO::build() {

    sync->out.link(inSync);
    sync->setRunOnHost(false);
    inSync.setBlocking(false);
    inSync.addCallback(std::bind(&BasaltVIO::stereoCB, this, std::placeholders::_1));
    imu.setMaxSize(0);
    imu.setBlocking(false);
    imu.addCallback(std::bind(&BasaltVIO::imuCB, this, std::placeholders::_1));

    basalt::PoseState<double>::SE3 initTrans(Eigen::Quaterniond::Identity(), Eigen::Vector3d(0, 0, 0));
    Eigen::Matrix<double, 3, 3> R180;
    R180 << -1.0, 0.0, 0.0, 0.0, -1.0, 0.0, 0.0, 0.0, 1.0;
    Eigen::Quaterniond q180(R180);
    basalt::PoseState<double>::SE3 opticalTransform180(q180, Eigen::Vector3d(0, 0, 0));
    // to output pose in FLU world coordinates
    localTransform = std::make_shared<basalt::PoseState<double>::SE3>(initTrans * opticalTransform180.inverse());

    return std::static_pointer_cast<BasaltVIO>(shared_from_this());
}

void BasaltVIO::setLocalTransform(const std::shared_ptr<TransformData>& transform) {
    auto trans = transform->getTranslation();
    auto quat = transform->getQuaternion();
    localTransform =
        std::make_shared<basalt::PoseState<double>::SE3>(Eigen::Quaterniond(quat.qw, quat.qx, quat.qy, quat.qz), Eigen::Vector3d(trans.x, trans.y, trans.z));
}
void BasaltVIO::run() {
    basalt::PoseVelBiasState<double>::Ptr data;
    Eigen::Matrix<double, 3, 3> R;
    R << 0.0, 0.0, 1.0, -1.0, 0.0, 0.0, 0.0, -1.0, 0.0;
    Eigen::Quaterniond q(R);
    basalt::PoseState<double>::SE3 opticalTransform(q, Eigen::Vector3d(0, 0, 0));

    while(isRunning()) {
        if(!calibrated) continue;
        pimpl->outStateQueue->pop(data);

        if(!data.get()) continue;
        basalt::PoseState<double>::SE3 pose = (*localTransform * data->T_w_i * calib->T_i_c[0]);

        // pose is in RDF orientation, convert to FLU
        auto finalPose = pose * opticalTransform.inverse();
        auto trans = finalPose.translation();
        auto rot = finalPose.unit_quaternion();
        auto out = std::make_shared<TransformData>(trans.x(), trans.y(), trans.z(), rot.x(), rot.y(), rot.z(), rot.w());
        transform.send(out);
        passthrough.send(leftImg);
    }
}

void BasaltVIO::stereoCB(std::shared_ptr<ADatatype> in) {
    auto group = std::dynamic_pointer_cast<MessageGroup>(in);
    if(group == nullptr) return;
    if(!calibrated) {
        std::vector<std::shared_ptr<ImgFrame>> imgFrames;
        for(auto& msg : *group) {
            imgFrames.emplace_back(std::dynamic_pointer_cast<ImgFrame>(msg.second));
        }

        initialize(imgFrames);
    }
    int i = 0;
    basalt::OpticalFlowInput::Ptr data(new basalt::OpticalFlowInput(2));
    for(auto& msg : *group) {
        std::shared_ptr<ImgFrame> imgFrame = std::dynamic_pointer_cast<ImgFrame>(msg.second);
        if(i == 0) {
            leftImg = imgFrame;
        };
        auto t = imgFrame->getTimestamp();
        int64_t tNS = std::chrono::time_point_cast<std::chrono::nanoseconds>(t).time_since_epoch().count();
        auto exposure = imgFrame->getExposureTime();

        int exposureMS = std::chrono::duration_cast<std::chrono::milliseconds>(exposure).count();
        data->img_data[i].img = std::make_shared<basalt::ManagedImage<uint16_t>>(imgFrame->getWidth(), imgFrame->getHeight());
        data->t_ns = tNS;
        data->img_data[i].exposure = exposureMS;
        size_t fullSize = imgFrame->getWidth() * imgFrame->getHeight();
        const uint8_t* dataIN = imgFrame->getData().data();
        uint16_t* data_out = data->img_data[i].img->ptr;
        for(size_t j = 0; j < fullSize; j++) {
            int val = dataIN[j];
            val = val << 8;
            data_out[j] = val;
        }
        i++;
    }
    lastImgData = data;
    if(pimpl->imageDataQueue) {
        pimpl->imageDataQueue->push(data);
    }
};

void BasaltVIO::imuCB(std::shared_ptr<ADatatype> imuData) {
    auto imuPackets = std::dynamic_pointer_cast<IMUData>(imuData);

    for(auto& imuPacket : imuPackets->packets) {
        basalt::ImuData<double>::Ptr data;
        data = std::make_shared<basalt::ImuData<double>>();
        auto t = imuPacket.acceleroMeter.getTimestamp();
        int64_t t_ns = std::chrono::time_point_cast<std::chrono::nanoseconds>(t).time_since_epoch().count();

        data->t_ns = t_ns;
        data->accel = Eigen::Vector3d(imuPacket.acceleroMeter.x, imuPacket.acceleroMeter.y, imuPacket.acceleroMeter.z);
        data->gyro = Eigen::Vector3d(imuPacket.gyroscope.x, imuPacket.gyroscope.y, imuPacket.gyroscope.z);
        if(pimpl->imuDataQueue) pimpl->imuDataQueue->push(data);
    }
};
void BasaltVIO::initialize(std::vector<std::shared_ptr<ImgFrame>> frames) {
    if(threadNum > 0) {
        pimpl->tbbGlobalControl = std::make_shared<tbb::global_control>(tbb::global_control::max_allowed_parallelism, threadNum);
    }

    auto pipeline = getParentPipeline();
    using Scalar = double;
    calib = std::make_shared<basalt::Calibration<Scalar>>();
    calib->imu_update_rate = imuUpdateRate;

    auto calibHandler = pipeline.getDefaultDevice()->readCalibration();

    for(const auto& frame : frames) {
        Eigen::Vector2i resolution;
        resolution << frame->getWidth(), frame->getHeight();
        calib->resolution.push_back(resolution);
        auto camID = static_cast<CameraBoardSocket>(frame->getInstanceNum());
        // imu extrinsics
        std::vector<std::vector<float>> imuExtr = calibHandler.getImuToCameraExtrinsics(camID, useSpecTranslation);

        Eigen::Matrix<Scalar, 3, 3> R;
        R << imuExtr[0][0], imuExtr[0][1], imuExtr[0][2], imuExtr[1][0], imuExtr[1][1], imuExtr[1][2], imuExtr[2][0], imuExtr[2][1], imuExtr[2][2];
        Eigen::Quaterniond q(R);

        Eigen::Vector3d trans(double(imuExtr[0][3]) * 0.01, double(imuExtr[1][3]) * 0.01, double(imuExtr[2][3]) * 0.01);
        basalt::Calibration<Scalar>::SE3 T_i_c(q, trans);
        calib->T_i_c.push_back(T_i_c);

        // camera intrinsics
        auto intrinsics = calibHandler.getCameraIntrinsics(camID, frame->getWidth(), frame->getHeight());
        auto model = calibHandler.getDistortionModel(camID);
        auto distCoeffs = calibHandler.getDistortionCoefficients(camID);
        basalt::GenericCamera<Scalar> camera;
        if(model == CameraModel::Perspective) {
            basalt::PinholeRadtan8Camera<Scalar>::VecN params;
            // fx, fy, cx, cy
            double fx = intrinsics[0][0];
            double fy = intrinsics[1][1];
            double cx = intrinsics[0][2];
            double cy = intrinsics[1][2];
            double k1 = distCoeffs[0];
            double k2 = distCoeffs[1];
            double p1 = distCoeffs[2];
            double p2 = distCoeffs[3];
            double k3 = distCoeffs[4];
            double k4 = distCoeffs[5];
            double k5 = distCoeffs[6];
            double k6 = distCoeffs[7];
            params << fx, fy, cx, cy, k1, k2, p1, p2, k3, k4, k5, k6;
            basalt::PinholeRadtan8Camera<Scalar> pinhole(params);
            camera.variant = pinhole;
        } else if(model == CameraModel::Fisheye) {
            // fx, fy, cx, cy
            double fx = intrinsics[0][0];
            double fy = intrinsics[1][1];
            double cx = intrinsics[0][2];
            double cy = intrinsics[1][2];
            double k1 = distCoeffs[0];
            double k2 = distCoeffs[1];
            double k3 = distCoeffs[2];
            double k4 = distCoeffs[3];
            basalt::KannalaBrandtCamera4<Scalar>::VecN params;
            params << fx, fy, cx, cy, k1, k2, k3, k4;
            basalt::KannalaBrandtCamera4<Scalar> kannala(params);
            camera.variant = kannala;
        } else {
            throw std::runtime_error("Unknown distortion model");
        }
        calib->intrinsics.push_back(camera);
    }
    vioConfig.load(configPath);
    optFlowPtr = basalt::OpticalFlowFactory::getOpticalFlow(vioConfig, *calib);
    optFlowPtr->start();
    pimpl->imageDataQueue = optFlowPtr->input_img_queue;
    vio = basalt::VioEstimatorFactory::getVioEstimator(vioConfig, *calib, basalt::constants::g, true, true);
    vio->initialize(Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero());
    pimpl->imuDataQueue = vio->imu_data_queue;
    optFlowPtr->output_queue = vio->vision_data_queue;
    pimpl->outStateQueue = std::make_shared<tbb::concurrent_bounded_queue<basalt::PoseVelBiasState<double>::Ptr>>();
    vio->out_state_queue = pimpl->outStateQueue;
    vio->opt_flow_depth_guess_queue = optFlowPtr->input_depth_queue;
    vio->opt_flow_state_queue = optFlowPtr->input_state_queue;
    calibrated = true;
}
}  // namespace node
}  // namespace dai