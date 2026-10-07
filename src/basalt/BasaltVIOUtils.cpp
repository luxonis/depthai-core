#include "BasaltVIOUtils.hpp"

#include <stdexcept>

namespace dai {
namespace utility {

basalt::GenericCamera<double> getBasaltCameraModel(const CalibrationHandler& calibHandler, const ImgFrame& frame) {
    using Scalar = double;
    const auto camID = static_cast<CameraBoardSocket>(frame.getInstanceNum());
    // camera intrinsics
    auto intrinsics = calibHandler.getCameraIntrinsics(camID, frame.getWidth(), frame.getHeight());
    auto model = calibHandler.getDistortionModel(camID);
    auto distCoeffs = calibHandler.getDistortionCoefficients(camID);
    if((model == CameraModel::Perspective && distCoeffs.size() < 8) || (model == CameraModel::Fisheye && distCoeffs.size() < 4)) {
        throw std::runtime_error("VIO camera calibration has insufficient distortion coefficients.");
    }
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
    return camera;
}

basalt::VioConfig getDefaultBasaltVIOConfig() {
    basalt::VioConfig config;
    config.optical_flow_type = "frame_to_frame";
    config.optical_flow_detection_grid_size = 50;
    config.optical_flow_detection_num_points_cell = 1;
    config.optical_flow_detection_min_threshold = 5;
    config.optical_flow_detection_max_threshold = 40;
    config.optical_flow_detection_nonoverlap = true;
    config.optical_flow_max_recovered_dist2 = 0.04;
    config.optical_flow_pattern = 51;
    config.optical_flow_max_iterations = 5;
    config.optical_flow_epipolar_error = 0.005;
    config.optical_flow_levels = 3;
    config.optical_flow_skip_frames = 1;
    config.optical_flow_matching_guess_type = basalt::MatchingGuessType::REPROJ_AVG_DEPTH;
    config.optical_flow_matching_default_depth = 2.0;
    config.optical_flow_image_safe_radius = 472.0;
    config.optical_flow_recall_enable = false;
    config.optical_flow_recall_all_cams = false;
    config.optical_flow_recall_num_points_cell = true;
    config.optical_flow_recall_over_tracking = false;
    config.optical_flow_recall_update_patch_viewpoint = false;
    config.optical_flow_recall_max_patch_dist = 3;
    config.optical_flow_recall_max_patch_norms = {1.74, 0.96, 0.99, 0.44};
    config.vio_linearization_type = basalt::LinearizationType::ABS_QR;
    config.vio_sqrt_marg = true;
    config.vio_max_states = 3;
    config.vio_max_kfs = 7;
    config.vio_min_frames_after_kf = 5;
    config.vio_new_kf_keypoints_thresh = 0.7;
    config.vio_debug = false;
    config.vio_extended_logging = false;
    config.vio_obs_std_dev = 0.5;
    config.vio_obs_huber_thresh = 1.0;
    config.vio_min_triangulation_dist = 0.05;
    config.vio_max_iterations = 7;
    config.vio_enforce_realtime = false;
    config.vio_use_lm = true;
    config.vio_lm_lambda_initial = 1e-4;
    config.vio_lm_lambda_min = 1e-6;
    config.vio_lm_lambda_max = 1e2;
    config.vio_scale_jacobian = false;
    config.vio_init_pose_weight = 1e8;
    config.vio_init_ba_weight = 1e1;
    config.vio_init_bg_weight = 1e2;
    config.vio_marg_lost_landmarks = true;
    config.vio_fix_long_term_keyframes = false;
    config.vio_kf_marg_feature_ratio = 0.1;
    config.vio_kf_marg_criteria = basalt::KeyframeMargCriteria::KF_MARG_DEFAULT;
    config.mapper_obs_std_dev = 0.25;
    config.mapper_obs_huber_thresh = 1.5;
    config.mapper_detection_num_points = 800;
    config.mapper_num_frames_to_match = 30;
    config.mapper_frames_to_match_threshold = 0.04;
    config.mapper_min_matches = 20;
    config.mapper_ransac_threshold = 5e-5;
    config.mapper_min_track_length = 5;
    config.mapper_max_hamming_distance = 70;
    config.mapper_second_best_test_ratio = 1.2;
    config.mapper_bow_num_bits = 16;
    config.mapper_min_triangulation_dist = 0.07;
    config.mapper_no_factor_weights = false;
    config.mapper_use_factors = true;
    config.mapper_use_lm = true;
    config.mapper_lm_lambda_min = 1e-32;
    config.mapper_lm_lambda_max = 1e3;
    return config;
}

}  // namespace utility
}  // namespace dai
