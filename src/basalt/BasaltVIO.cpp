#include "depthai/basalt/BasaltVIO.hpp"
#include <basalt/utils/common_types.h>

#include "../utility/PimplImpl.hpp"
#include "basalt/calibration/calibration.hpp"
#include "basalt/serialization/headers_serialization.h"
#include "basalt/spline/se3_spline.h"
#include "basalt/utils/vio_config.h"
#include "basalt/vi_estimator/vio_estimator.h"
#include "depthai/pipeline/Pipeline.hpp"
#include "depthai/pipeline/ThreadedHostNode.hpp"
#include "depthai/pipeline/datatype/MessageGroup.hpp"
#include "pipeline/datatype/TransformData.hpp"
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
    std::shared_ptr<basalt::Calibration<double>> calib;

    basalt::OpticalFlowBase::Ptr optFlowPtr;
    basalt::VioEstimatorBase::Ptr vio;
    basalt::OpticalFlowInput::Ptr lastImgData;
    /**
     * VIO configuration file.
     */
    basalt::VioConfig vioConfig;

    std::vector<int64_t> vioTNSec;
    std::shared_ptr<basalt::PoseState<double>::SE3> localTransform;
};

BasaltVIO::BasaltVIO() {}

BasaltVIO::~BasaltVIO() = default;

void BasaltVIO::buildInternal() {
    sync->out.link(inSync);
    inSync.addCallback(std::bind(&BasaltVIO::stereoCB, this, std::placeholders::_1));
    imu.addCallback(std::bind(&BasaltVIO::imuCB, this, std::placeholders::_1));

    basalt::PoseState<double>::SE3 initTrans(Eigen::Quaterniond::Identity(), Eigen::Vector3d(0, 0, 0));
    Eigen::Matrix<double, 3, 3> R;
    R << 0.0, -1.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0;
    Eigen::Quaterniond q(R);
    basalt::PoseState<double>::SE3 initialRotation(q, Eigen::Vector3d(0, 0, 0));
    // to output pose in FLU world coordinates
    pimpl->localTransform = std::make_shared<basalt::PoseState<double>::SE3>(initTrans * initialRotation.inverse());
    setDefaultVIOConfig();
}

void BasaltVIO::setLocalTransform(const std::shared_ptr<TransformData>& transform) {
    auto trans = transform->getTranslation();
    auto quat = transform->getQuaternion();
    pimpl->localTransform =
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
        basalt::PoseState<double>::SE3 pose = (*pimpl->localTransform * data->T_w_i * pimpl->calib->T_i_c[0]);

        // pose is in RDF orientation, convert to FLU
        auto finalPose = pose * opticalTransform.inverse();
        auto trans = finalPose.translation();
        auto rot = finalPose.unit_quaternion();
        auto out = std::make_shared<TransformData>(trans.x(), trans.y(), trans.z(), rot.x(), rot.y(), rot.z(), rot.w());
        transform.send(out);
        std::lock_guard<std::mutex> lck(imgMtx);
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
            std::lock_guard<std::mutex> lck(imgMtx);
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
    pimpl->lastImgData = data;
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

void BasaltVIO::setImuExtrinsics(const std::shared_ptr<TransformData>& imuExtr) {
    imuExtrinsics = imuExtr;
}

void BasaltVIO::setAccelBias(const std::vector<double>& accelBias) {
    if(accelBias.size() != 9) {
        throw std::invalid_argument("Accelerometer bias vector must have 9 elements.");
    }
    this->accelBias = accelBias;
}

void BasaltVIO::setAccelNoiseStd(const std::vector<double>& accelNoiseStd) {
    if(accelNoiseStd.size() != 3) {
        throw std::invalid_argument("Accelerometer noise vector must have 3 elements.");
    }
    this->accelNoiseStd = accelNoiseStd;
    ;
}

void BasaltVIO::setGyroNoiseStd(const std::vector<double>& gyroNoiseStd) {
    if(gyroNoiseStd.size() != 3) {
        throw std::invalid_argument("Gyroscope noise vector must have 3 elements.");
    }
    this->gyroNoiseStd = gyroNoiseStd;
}

void BasaltVIO::setGyroBias(const std::vector<double>& gyroBias) {
    if(gyroBias.size() != 12) {
        throw std::invalid_argument("Gyroscope bias vector must have 9 elements.");
    }

    this->gyroBias = gyroBias;
}

void BasaltVIO::initialize(std::vector<std::shared_ptr<ImgFrame>> frames) {
    if(threadNum > 0) {
        pimpl->tbbGlobalControl = std::make_shared<tbb::global_control>(tbb::global_control::max_allowed_parallelism, threadNum);
    }

    auto pipeline = getParentPipeline();
    using Scalar = double;
    pimpl->calib = std::make_shared<basalt::Calibration<Scalar>>();
    pimpl->calib->imu_update_rate = imuUpdateRate;

    auto calibHandler = pipeline.getDefaultDevice()->readCalibration();

    for(const auto& frame : frames) {
        Eigen::Vector2i resolution;
        resolution << frame->getWidth(), frame->getHeight();
        pimpl->calib->resolution.push_back(resolution);
        auto camID = static_cast<CameraBoardSocket>(frame->getInstanceNum());
        // imu extrinsics
        if(imuExtrinsics.has_value()) {
            Eigen::Vector3d trans(
                imuExtrinsics.value()->getTranslation().x, imuExtrinsics.value()->getTranslation().y, imuExtrinsics.value()->getTranslation().z);
            Eigen::Quaterniond q(imuExtrinsics.value()->getQuaternion().qw,
                                 imuExtrinsics.value()->getQuaternion().qx,
                                 imuExtrinsics.value()->getQuaternion().qy,
                                 imuExtrinsics.value()->getQuaternion().qz);
            basalt::Calibration<Scalar>::SE3 T_i_c(q, trans);
            pimpl->calib->T_i_c.push_back(T_i_c);
        } else {
            std::vector<std::vector<float>> imuExtr = calibHandler.getCameraToImuExtrinsics(camID, useSpecTranslation);

            Eigen::Matrix<Scalar, 3, 3> R;
            R << double(imuExtr[0][0]), double(imuExtr[0][1]), double(imuExtr[0][2]), double(imuExtr[1][0]), double(imuExtr[1][1]), double(imuExtr[1][2]),
                double(imuExtr[2][0]), double(imuExtr[2][1]), double(imuExtr[2][2]);
            Eigen::Quaterniond q(R);

            Eigen::Vector3d trans(double(imuExtr[0][3]) * 0.01, double(imuExtr[1][3]) * 0.01, double(imuExtr[2][3]) * 0.01);
            basalt::Calibration<Scalar>::SE3 T_i_c(q, trans);
            pimpl->calib->T_i_c.push_back(T_i_c);
        }
        if(accelBias.has_value()) {
            Eigen::Matrix<Scalar, 9, 1> accelBiasFull;

            accelBiasFull << accelBias.value()[3] + 1, 0, 0, accelBias.value()[4], accelBias.value()[6] + 1, 0, accelBias.value()[5], accelBias.value()[7],
                accelBias.value()[8] + 1;
            basalt::CalibAccelBias<Scalar> accel_bias;
            accel_bias.getParam() = accelBiasFull;
            pimpl->calib->calib_accel_bias = accel_bias;
        }
        if(gyroBias.has_value()) {
            Eigen::Matrix<Scalar, 12, 1> gyroBiasFull;

            gyroBiasFull << gyroBias.value()[3] + 1, gyroBias.value()[4], gyroBias.value()[5], gyroBias.value()[6], gyroBias.value()[7] + 1,
                gyroBias.value()[8], gyroBias.value()[9], gyroBias.value()[10], gyroBias.value()[11] + 1;
            basalt::CalibGyroBias<Scalar> gyro_bias;
            gyro_bias.getParam() = gyroBiasFull;
            pimpl->calib->calib_gyro_bias = gyro_bias;
        }
        if(accelNoiseStd.has_value()) {
            pimpl->calib->accel_noise_std = Eigen::Vector3d(accelNoiseStd.value()[0], accelNoiseStd.value()[1], accelNoiseStd.value()[2]).cwiseSqrt();
        }
        if(gyroNoiseStd.has_value()) {
            pimpl->calib->gyro_noise_std = Eigen::Vector3d(gyroNoiseStd.value()[0], gyroNoiseStd.value()[1], gyroNoiseStd.value()[2]).cwiseSqrt();
        }

        // camera intrinsics
        auto intrinsics = calibHandler.getCameraIntrinsics(camID, frame->getWidth(), frame->getHeight());
        auto model = calibHandler.getDistortionModel(camID);
        auto distCoeffs = calibHandler.getDistortionCoefficients(camID);
        basalt::GenericCamera<Scalar> camera;
        if(model == CameraModel::Perspective) {
            basalt::PinholeRadtan8Camera<Scalar>::VecN params;
            // fx, fy, cx, cy
            double fx = double(intrinsics[0][0]);
            double fy = double(intrinsics[1][1]);
            double cx = double(intrinsics[0][2]);
            double cy = double(intrinsics[1][2]);
            double k1 = double(distCoeffs[0]);
            double k2 = double(distCoeffs[1]);
            double p1 = double(distCoeffs[2]);
            double p2 = double(distCoeffs[3]);
            double k3 = double(distCoeffs[4]);
            double k4 = double(distCoeffs[5]);
            double k5 = double(distCoeffs[6]);
            double k6 = double(distCoeffs[7]);
            params << fx, fy, cx, cy, k1, k2, p1, p2, k3, k4, k5, k6;
            basalt::PinholeRadtan8Camera<Scalar> pinhole(params);
            camera.variant = pinhole;
        } else if(model == CameraModel::Fisheye) {
            // fx, fy, cx, cy
            double fx = double(intrinsics[0][0]);
            double fy = double(intrinsics[1][1]);
            double cx = double(intrinsics[0][2]);
            double cy = double(intrinsics[1][2]);
            double k1 = double(distCoeffs[0]);
            double k2 = double(distCoeffs[1]);
            double k3 = double(distCoeffs[2]);
            double k4 = double(distCoeffs[3]);
            basalt::KannalaBrandtCamera4<Scalar>::VecN params;
            params << fx, fy, cx, cy, k1, k2, k3, k4;
            basalt::KannalaBrandtCamera4<Scalar> kannala(params);
            camera.variant = kannala;
        } else {
            throw std::runtime_error("Unknown distortion model");
        }
        pimpl->calib->intrinsics.push_back(camera);
    }
    if(!configPath.empty()) {
        pimpl->vioConfig.load(configPath);
    }

    pimpl->optFlowPtr = basalt::OpticalFlowFactory::getOpticalFlow(pimpl->vioConfig, *pimpl->calib);
    pimpl->optFlowPtr->show_gui = false;
    pimpl->optFlowPtr->start();
    pimpl->imageDataQueue = pimpl->optFlowPtr->input_img_queue;
    pimpl->vio = basalt::VioEstimatorFactory::getVioEstimator(pimpl->vioConfig, *pimpl->calib, basalt::constants::g, true, true);
    pimpl->vio->initialize(Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero());
    pimpl->imuDataQueue = pimpl->vio->imu_data_queue;
    pimpl->optFlowPtr->output_queue = pimpl->vio->vision_data_queue;
    pimpl->outStateQueue = std::make_shared<tbb::concurrent_bounded_queue<basalt::PoseVelBiasState<double>::Ptr>>();
    pimpl->vio->out_state_queue = pimpl->outStateQueue;
    pimpl->vio->opt_flow_depth_guess_queue = pimpl->optFlowPtr->input_depth_queue;
    pimpl->vio->opt_flow_state_queue = pimpl->optFlowPtr->input_state_queue;
    pimpl->vio->opt_flow_lm_bundle_queue = pimpl->optFlowPtr->input_lm_bundle_queue;
    initialized = true;
}
void BasaltVIO::runSyncOnHost(bool runOnHost) {
    sync->setRunOnHost(runOnHost);
}
void BasaltVIO::setConfig(const BasaltVIO::VioConfig& config){
    pimpl->vioConfig.optical_flow_type = config.optical_flow_type;
    pimpl->vioConfig.optical_flow_detection_grid_size = config.optical_flow_detection_grid_size;
    pimpl->vioConfig.optical_flow_detection_num_points_cell = config.optical_flow_detection_num_points_cell;
    pimpl->vioConfig.optical_flow_detection_min_threshold = config.optical_flow_detection_min_threshold;
    pimpl->vioConfig.optical_flow_detection_max_threshold = config.optical_flow_detection_max_threshold;
    pimpl->vioConfig.optical_flow_detection_nonoverlap = config.optical_flow_detection_nonoverlap;
    pimpl->vioConfig.optical_flow_max_recovered_dist2 = config.optical_flow_max_recovered_dist2;
    pimpl->vioConfig.optical_flow_pattern = config.optical_flow_pattern;
    pimpl->vioConfig.optical_flow_max_iterations = config.optical_flow_max_iterations;
    pimpl->vioConfig.optical_flow_epipolar_error = config.optical_flow_epipolar_error;
    pimpl->vioConfig.optical_flow_levels = config.optical_flow_levels;
    pimpl->vioConfig.optical_flow_skip_frames = config.optical_flow_skip_frames;
    pimpl->vioConfig.optical_flow_matching_guess_type = static_cast<basalt::MatchingGuessType>(config.optical_flow_matching_guess_type);
    pimpl->vioConfig.optical_flow_matching_default_depth = config.optical_flow_matching_default_depth;
    pimpl->vioConfig.optical_flow_image_safe_radius = config.optical_flow_image_safe_radius;
    pimpl->vioConfig.optical_flow_recall_enable = config.optical_flow_recall_enable;
    pimpl->vioConfig.optical_flow_recall_all_cams = config.optical_flow_recall_all_cams;
    pimpl->vioConfig.optical_flow_recall_num_points_cell = config.optical_flow_recall_num_points_cell;
    pimpl->vioConfig.optical_flow_recall_over_tracking = config.optical_flow_recall_over_tracking;
    pimpl->vioConfig.optical_flow_recall_update_patch_viewpoint = config.optical_flow_recall_update_patch_viewpoint;
    pimpl->vioConfig.optical_flow_recall_max_patch_dist = config.optical_flow_recall_max_patch_dist;
    pimpl->vioConfig.optical_flow_recall_max_patch_norms = config.optical_flow_recall_max_patch_norms;
    pimpl->vioConfig.vio_linearization_type = static_cast<basalt::LinearizationType>(config.vio_linearization_type);
    pimpl->vioConfig.vio_sqrt_marg = config.vio_sqrt_marg;
    pimpl->vioConfig.vio_max_states = config.vio_max_states;
    pimpl->vioConfig.vio_max_kfs = config.vio_max_kfs;
    pimpl->vioConfig.vio_min_frames_after_kf = config.vio_min_frames_after_kf;
    pimpl->vioConfig.vio_new_kf_keypoints_thresh = config.vio_new_kf_keypoints_thresh;
    pimpl->vioConfig.vio_debug = config.vio_debug;
    pimpl->vioConfig.vio_extended_logging = config.vio_extended_logging;
    pimpl->vioConfig.vio_obs_std_dev = config.vio_obs_std_dev;
    pimpl->vioConfig.vio_obs_huber_thresh = config.vio_obs_huber_thresh;
    pimpl->vioConfig.vio_min_triangulation_dist = config.vio_min_triangulation_dist;
    pimpl->vioConfig.vio_max_iterations = config.vio_max_iterations;
    pimpl->vioConfig.vio_enforce_realtime = config.vio_enforce_realtime;
    pimpl->vioConfig.vio_use_lm = config.vio_use_lm;
    pimpl->vioConfig.vio_lm_lambda_initial = config.vio_lm_lambda_initial;
    pimpl->vioConfig.vio_lm_lambda_min = config.vio_lm_lambda_min;
    pimpl->vioConfig.vio_lm_lambda_max = config.vio_lm_lambda_max;
    pimpl->vioConfig.vio_scale_jacobian = config.vio_scale_jacobian;
    pimpl->vioConfig.vio_init_pose_weight = config.vio_init_pose_weight;
    pimpl->vioConfig.vio_init_ba_weight = config.vio_init_ba_weight;
    pimpl->vioConfig.vio_init_bg_weight = config.vio_init_bg_weight;
    pimpl->vioConfig.vio_marg_lost_landmarks = config.vio_marg_lost_landmarks;
    pimpl->vioConfig.vio_fix_long_term_keyframes = config.vio_fix_long_term_keyframes;
    pimpl->vioConfig.vio_kf_marg_feature_ratio = config.vio_kf_marg_feature_ratio;
    pimpl->vioConfig.vio_kf_marg_criteria = static_cast<basalt::KeyframeMargCriteria>(config.vio_kf_marg_criteria);
    pimpl->vioConfig.mapper_obs_std_dev = config.mapper_obs_std_dev;
    pimpl->vioConfig.mapper_obs_huber_thresh = config.mapper_obs_huber_thresh;
    pimpl->vioConfig.mapper_detection_num_points = config.mapper_detection_num_points;
    pimpl->vioConfig.mapper_num_frames_to_match = config.mapper_num_frames_to_match;
    pimpl->vioConfig.mapper_frames_to_match_threshold = config.mapper_frames_to_match_threshold;
    pimpl->vioConfig.mapper_min_matches = config.mapper_min_matches;
    pimpl->vioConfig.mapper_ransac_threshold = config.mapper_ransac_threshold;
    pimpl->vioConfig.mapper_min_track_length = config.mapper_min_track_length;
    pimpl->vioConfig.mapper_max_hamming_distance = config.mapper_max_hamming_distance;
    pimpl->vioConfig.mapper_second_best_test_ratio = config.mapper_second_best_test_ratio;
    pimpl->vioConfig.mapper_bow_num_bits = config.mapper_bow_num_bits;
    pimpl->vioConfig.mapper_min_triangulation_dist = config.mapper_min_triangulation_dist;
    pimpl->vioConfig.mapper_no_factor_weights = config.mapper_no_factor_weights;
    pimpl->vioConfig.mapper_use_factors = config.mapper_use_factors;
    pimpl->vioConfig.mapper_use_lm = config.mapper_use_lm;
    pimpl->vioConfig.mapper_lm_lambda_min = config.mapper_lm_lambda_min;
    pimpl->vioConfig.mapper_lm_lambda_max = config.mapper_lm_lambda_max;
}
void BasaltVIO::setDefaultVIOConfig() {
    pimpl->vioConfig.optical_flow_type = "frame_to_frame";
    pimpl->vioConfig.optical_flow_detection_grid_size = 50;
    pimpl->vioConfig.optical_flow_detection_num_points_cell = 1;
    pimpl->vioConfig.optical_flow_detection_min_threshold = 5;
    pimpl->vioConfig.optical_flow_detection_max_threshold = 40;
    pimpl->vioConfig.optical_flow_detection_nonoverlap = true;
    pimpl->vioConfig.optical_flow_max_recovered_dist2 = 0.04;
    pimpl->vioConfig.optical_flow_pattern = 51;
    pimpl->vioConfig.optical_flow_max_iterations = 5;
    pimpl->vioConfig.optical_flow_epipolar_error = 0.005;
    pimpl->vioConfig.optical_flow_levels = 3;
    pimpl->vioConfig.optical_flow_skip_frames = 1;
    pimpl->vioConfig.optical_flow_matching_guess_type = basalt::MatchingGuessType::REPROJ_AVG_DEPTH;
    pimpl->vioConfig.optical_flow_matching_default_depth = 2.0;
    pimpl->vioConfig.optical_flow_image_safe_radius = 472.0;
    pimpl->vioConfig.optical_flow_recall_enable = false;
    pimpl->vioConfig.optical_flow_recall_all_cams = false;
    pimpl->vioConfig.optical_flow_recall_num_points_cell = true;
    pimpl->vioConfig.optical_flow_recall_over_tracking = false;
    pimpl->vioConfig.optical_flow_recall_update_patch_viewpoint = false;
    pimpl->vioConfig.optical_flow_recall_max_patch_dist = 3;
    pimpl->vioConfig.optical_flow_recall_max_patch_norms = {1.74, 0.96, 0.99, 0.44};
    pimpl->vioConfig.vio_linearization_type = basalt::LinearizationType::ABS_QR;
    pimpl->vioConfig.vio_sqrt_marg = true;
    pimpl->vioConfig.vio_max_states = 3;
    pimpl->vioConfig.vio_max_kfs = 7;
    pimpl->vioConfig.vio_min_frames_after_kf = 5;
    pimpl->vioConfig.vio_new_kf_keypoints_thresh = 0.7;
    pimpl->vioConfig.vio_debug = false;
    pimpl->vioConfig.vio_extended_logging = false;
    pimpl->vioConfig.vio_obs_std_dev = 0.5;
    pimpl->vioConfig.vio_obs_huber_thresh = 1.0;
    pimpl->vioConfig.vio_min_triangulation_dist = 0.05;
    pimpl->vioConfig.vio_max_iterations = 7;
    pimpl->vioConfig.vio_enforce_realtime = false;
    pimpl->vioConfig.vio_use_lm = true;
    pimpl->vioConfig.vio_lm_lambda_initial = 1e-4;
    pimpl->vioConfig.vio_lm_lambda_min = 1e-6;
    pimpl->vioConfig.vio_lm_lambda_max = 1e2;
    pimpl->vioConfig.vio_scale_jacobian = false;
    pimpl->vioConfig.vio_init_pose_weight = 1e8;
    pimpl->vioConfig.vio_init_ba_weight = 1e1;
    pimpl->vioConfig.vio_init_bg_weight = 1e2;
    pimpl->vioConfig.vio_marg_lost_landmarks = true;
    pimpl->vioConfig.vio_fix_long_term_keyframes = false;
    pimpl->vioConfig.vio_kf_marg_feature_ratio = 0.1;
    pimpl->vioConfig.vio_kf_marg_criteria = basalt::KeyframeMargCriteria::KF_MARG_DEFAULT;
    pimpl->vioConfig.mapper_obs_std_dev = 0.25;
    pimpl->vioConfig.mapper_obs_huber_thresh = 1.5;
    pimpl->vioConfig.mapper_detection_num_points = 800;
    pimpl->vioConfig.mapper_num_frames_to_match = 30;
    pimpl->vioConfig.mapper_frames_to_match_threshold = 0.04;
    pimpl->vioConfig.mapper_min_matches = 20;
    pimpl->vioConfig.mapper_ransac_threshold = 5e-5;
    pimpl->vioConfig.mapper_min_track_length = 5;
    pimpl->vioConfig.mapper_max_hamming_distance = 70;
    pimpl->vioConfig.mapper_second_best_test_ratio = 1.2;
    pimpl->vioConfig.mapper_bow_num_bits = 16;
    pimpl->vioConfig.mapper_min_triangulation_dist = 0.07;
    pimpl->vioConfig.mapper_no_factor_weights = false;
    pimpl->vioConfig.mapper_use_factors = true;
    pimpl->vioConfig.mapper_use_lm = true;
    pimpl->vioConfig.mapper_lm_lambda_min = 1e-32;
    pimpl->vioConfig.mapper_lm_lambda_max = 1e3;
}
}  // namespace node
}  // namespace dai
