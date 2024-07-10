#include "depthai/basalt/BasaltVIO.hpp"

#include "../utility/PimplImpl.hpp"
#include "depthai/pipeline/Pipeline.hpp"
#include "depthai/pipeline/ThreadedHostNode.hpp"
#include "depthai/pipeline/datatype/MessageGroup.hpp"
#include "tbb/concurrent_queue.h"
#include "tbb/global_control.h"
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

BasaltVIO::BasaltVIO() {}
BasaltVIO::~BasaltVIO() = default;

std::shared_ptr<BasaltVIO> BasaltVIO::build() {
    sync->out.link(inSync);
    sync->setRunOnHost(false);
    inSync.addCallback(std::bind(&BasaltVIO::stereoCB, this, std::placeholders::_1));
    imu.addCallback(std::bind(&BasaltVIO::imuCB, this, std::placeholders::_1));

    basalt::PoseState<double>::SE3 initTrans(Eigen::Quaterniond::Identity(), Eigen::Vector3d(0, 0, 0));
    Eigen::Matrix<double, 3, 3> R180;
    R180 << -1.0, 0.0, 0.0, 0.0, -1.0, 0.0, 0.0, 0.0, 1.0;
    Eigen::Quaterniond q180(R180);
    basalt::PoseState<double>::SE3 opticalTransform180(q180, Eigen::Vector3d(0, 0, 0));
    // to output pose in FLU world coordinates
    localTransform = std::make_shared<basalt::PoseState<double>::SE3>(initTrans * opticalTransform180.inverse());
    setDefaultVIOConfig();
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
        if(!initialized) continue;
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
    if(!initialized) {
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

void BasaltVIO::stop() {
    pimpl->imageDataQueue->push(nullptr);
    pimpl->imuDataQueue->push(nullptr);
    ThreadedHostNode::stop();
}

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
    if(!configPath.empty()) {
        vioConfig.load(configPath);
    }

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
    initialized = true;
}

void BasaltVIO::setDefaultVIOConfig() {
    vioConfig.optical_flow_type = "frame_to_frame";
    vioConfig.optical_flow_detection_grid_size = 50;
    vioConfig.optical_flow_detection_num_points_cell = 1;
    vioConfig.optical_flow_detection_min_threshold = 5;
    vioConfig.optical_flow_detection_max_threshold = 40;
    vioConfig.optical_flow_detection_nonoverlap = true;
    vioConfig.optical_flow_max_recovered_dist2 = 0.04;
    vioConfig.optical_flow_pattern = 51;
    vioConfig.optical_flow_max_iterations = 5;
    vioConfig.optical_flow_epipolar_error = 0.005;
    vioConfig.optical_flow_levels = 3;
    vioConfig.optical_flow_skip_frames = 1;
    vioConfig.optical_flow_matching_guess_type = basalt::MatchingGuessType::REPROJ_AVG_DEPTH;
    vioConfig.optical_flow_matching_default_depth = 2.0;
    vioConfig.optical_flow_image_safe_radius = 472.0;
    vioConfig.optical_flow_recall_enable = false;
    vioConfig.optical_flow_recall_all_cams = false;
    vioConfig.optical_flow_recall_num_points_cell = true;
    vioConfig.optical_flow_recall_over_tracking = false;
    vioConfig.optical_flow_recall_update_patch_viewpoint = false;
    vioConfig.optical_flow_recall_max_patch_dist = 3;
    vioConfig.optical_flow_recall_max_patch_norms = {1.74, 0.96, 0.99, 0.44};
    vioConfig.vio_linearization_type = basalt::LinearizationType::ABS_QR;
    vioConfig.vio_sqrt_marg = true;
    vioConfig.vio_max_states = 3;
    vioConfig.vio_max_kfs = 7;
    vioConfig.vio_min_frames_after_kf = 5;
    vioConfig.vio_new_kf_keypoints_thresh = 0.7;
    vioConfig.vio_debug = false;
    vioConfig.vio_extended_logging = false;
    vioConfig.vio_obs_std_dev = 0.5;
    vioConfig.vio_obs_huber_thresh = 1.0;
    vioConfig.vio_min_triangulation_dist = 0.05;
    vioConfig.vio_max_iterations = 7;
    vioConfig.vio_enforce_realtime = false;
    vioConfig.vio_use_lm = true;
    vioConfig.vio_lm_lambda_initial = 1e-4;
    vioConfig.vio_lm_lambda_min = 1e-6;
    vioConfig.vio_lm_lambda_max = 1e2;
    vioConfig.vio_scale_jacobian = false;
    vioConfig.vio_init_pose_weight = 1e8;
    vioConfig.vio_init_ba_weight = 1e1;
    vioConfig.vio_init_bg_weight = 1e2;
    vioConfig.vio_marg_lost_landmarks = true;
    vioConfig.vio_fix_long_term_keyframes = false;
    vioConfig.vio_kf_marg_feature_ratio = 0.1;
    vioConfig.vio_kf_marg_criteria = basalt::KeyframeMargCriteria::KF_MARG_DEFAULT;
    vioConfig.mapper_obs_std_dev = 0.25;
    vioConfig.mapper_obs_huber_thresh = 1.5;
    vioConfig.mapper_detection_num_points = 800;
    vioConfig.mapper_num_frames_to_match = 30;
    vioConfig.mapper_frames_to_match_threshold = 0.04;
    vioConfig.mapper_min_matches = 20;
    vioConfig.mapper_ransac_threshold = 5e-5;
    vioConfig.mapper_min_track_length = 5;
    vioConfig.mapper_max_hamming_distance = 70;
    vioConfig.mapper_second_best_test_ratio = 1.2;
    vioConfig.mapper_bow_num_bits = 16;
    vioConfig.mapper_min_triangulation_dist = 0.07;
    vioConfig.mapper_no_factor_weights = false;
    vioConfig.mapper_use_factors = true;
    vioConfig.mapper_use_lm = true;
    vioConfig.mapper_lm_lambda_min = 1e-32;
    vioConfig.mapper_lm_lambda_max = 1e3;
}
}  // namespace node
}  // namespace dai