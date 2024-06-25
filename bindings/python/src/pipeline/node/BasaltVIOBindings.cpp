#include "Common.hpp"
#include "NodeBindings.hpp"
#include "depthai/pipeline/ThreadedHostNode.hpp"
#include "depthai/basalt/BasaltVIO.hpp"

#include <pybind11/eval.h>

extern py::handle daiNodeModule;

void bind_basaltnode(pybind11::module& m, void* pCallstack){
    using namespace dai;
    using namespace dai::node;

    // declare upfront
    auto basaltNode = ADD_NODE_DERIVED(BasaltVIO, ThreadedHostNode);
    py::class_<basalt::VioConfig> vioConfig(m, "VioConfig", DOC(dai, VioConfig));
    ///////////////////////////////////////////////////////////////////////
    ///////////////////////////////////////////////////////////////////////
    ///////////////////////////////////////////////////////////////////////
    // Call the rest of the type defines, then perform the actual bindings
    Callstack* callstack = (Callstack*) pCallstack;
    auto cb = callstack->top();
    callstack->pop();
    cb(m, pCallstack);
    // Actual bindings
    ///////////////////////////////////////////////////////////////////////
    ///////////////////////////////////////////////////////////////////////
    ///////////////////////////////////////////////////////////////////////
    // BasaltVIO Node
    basaltNode
        .def_property_readonly("left", [](BasaltVIO& node){ return &node.left; }, py::return_value_policy::reference_internal)
        .def_property_readonly("right", [](BasaltVIO& node){ return &node.right; }, py::return_value_policy::reference_internal)
        .def_readonly("imu", &BasaltVIO::imu, DOC(dai, node, BasaltVIO, imu))
        .def_readonly("transform", &BasaltVIO::transform, DOC(dai, node, BasaltVIO, transform))
        .def_readonly("passthrough", &BasaltVIO::passthrough, DOC(dai, node, BasaltVIO, passthrough))
        .def("build", &BasaltVIO::build, DOC(dai, node, BasaltVIO, build))
        .def("setImuUpdateRate", &BasaltVIO::setImuUpdateRate, py::arg("rate"), DOC(dai, node, BasaltVIO, setImuUpdateRate))
        .def("setConfigPath", &BasaltVIO::setConfigPath, py::arg("path"), DOC(dai, node, BasaltVIO, setConfigPath))
        .def_readwrite("vioConfig", &BasaltVIO::vioConfig, DOC(dai, node, BasaltVIO, vioConfig))
        .def("setLocalTransform", &BasaltVIO::setLocalTransform, py::arg("transform"), DOC(dai, node, BasaltVIO, setLocalTransform));
    // VioConfig
    vioConfig
    .def(py::init<>(), DOC(dai, VioConfig, VioConfig))
    .def_readwrite("optical_flow_type", &basalt::VioConfig::optical_flow_type, DOC(dai, VioConfig, optical_flow_type))
    .def_readwrite("optical_flow_detection_grid_size", &basalt::VioConfig::optical_flow_detection_grid_size, DOC(dai, VioConfig, optical_flow_detection_grid_size))
    .def_readwrite("optical_flow_detection_num_points_cell", &basalt::VioConfig::optical_flow_detection_num_points_cell, DOC(dai, VioConfig, optical_flow_detection_num_points_cell))
    .def_readwrite("optical_flow_detection_min_threshold", &basalt::VioConfig::optical_flow_detection_min_threshold, DOC(dai, VioConfig, optical_flow_detection_min_threshold))
    .def_readwrite("optical_flow_detection_max_threshold", &basalt::VioConfig::optical_flow_detection_max_threshold, DOC(dai, VioConfig, optical_flow_detection_max_threshold))
    .def_readwrite("optical_flow_detection_nonoverlap", &basalt::VioConfig::optical_flow_detection_nonoverlap, DOC(dai, VioConfig, optical_flow_detection_nonoverlap))
    .def_readwrite("optical_flow_max_recovered_dist2", &basalt::VioConfig::optical_flow_max_recovered_dist2, DOC(dai, VioConfig, optical_flow_max_recovered_dist2))
    .def_readwrite("optical_flow_pattern", &basalt::VioConfig::optical_flow_pattern, DOC(dai, VioConfig, optical_flow_pattern))
    .def_readwrite("optical_flow_max_iterations", &basalt::VioConfig::optical_flow_max_iterations, DOC(dai, VioConfig, optical_flow_max_iterations))
    .def_readwrite("optical_flow_epipolar_error", &basalt::VioConfig::optical_flow_epipolar_error, DOC(dai, VioConfig, optical_flow_epipolar_error))
    .def_readwrite("optical_flow_levels", &basalt::VioConfig::optical_flow_levels, DOC(dai, VioConfig, optical_flow_levels))
    .def_readwrite("optical_flow_skip_frames", &basalt::VioConfig::optical_flow_skip_frames, DOC(dai, VioConfig, optical_flow_skip_frames))
    .def_readwrite("optical_flow_matching_guess_type", &basalt::VioConfig::optical_flow_matching_guess_type, DOC(dai, VioConfig, optical_flow_matching_guess_type))
    .def_readwrite("optical_flow_matching_default_depth", &basalt::VioConfig::optical_flow_matching_default_depth, DOC(dai, VioConfig, optical_flow_matching_default_depth))
    .def_readwrite("optical_flow_image_safe_radius", &basalt::VioConfig::optical_flow_image_safe_radius, DOC(dai, VioConfig, optical_flow_image_safe_radius))
    .def_readwrite("optical_flow_recall_enable", &basalt::VioConfig::optical_flow_recall_enable, DOC(dai, VioConfig, optical_flow_recall_enable))
    .def_readwrite("optical_flow_recall_all_cams", &basalt::VioConfig::optical_flow_recall_all_cams, DOC(dai, VioConfig, optical_flow_recall_all_cams))
    .def_readwrite("optical_flow_recall_num_points_cell", &basalt::VioConfig::optical_flow_recall_num_points_cell, DOC(dai, VioConfig, optical_flow_recall_num_points_cell))
    .def_readwrite("optical_flow_recall_over_tracking", &basalt::VioConfig::optical_flow_recall_over_tracking, DOC(dai, VioConfig, optical_flow_recall_over_tracking))
    .def_readwrite("optical_flow_recall_update_patch_viewpoint", &basalt::VioConfig::optical_flow_recall_update_patch_viewpoint, DOC(dai, VioConfig, optical_flow_recall_update_patch_viewpoint))
    .def_readwrite("optical_flow_recall_max_patch_dist", &basalt::VioConfig::optical_flow_recall_max_patch_dist, DOC(dai, VioConfig, optical_flow_recall_max_patch_dist))
    .def_readwrite("optical_flow_recall_max_patch_norms", &basalt::VioConfig::optical_flow_recall_max_patch_norms, DOC(dai, VioConfig, optical_flow_recall_max_patch_norms))
    .def_readwrite("vio_linearization_type", &basalt::VioConfig::vio_linearization_type, DOC(dai, VioConfig, vio_linearization_type))
    .def_readwrite("vio_sqrt_marg", &basalt::VioConfig::vio_sqrt_marg, DOC(dai, VioConfig, vio_sqrt_marg))
    .def_readwrite("vio_max_states", &basalt::VioConfig::vio_max_states, DOC(dai, VioConfig, vio_max_states))
    .def_readwrite("vio_max_kfs", &basalt::VioConfig::vio_max_kfs, DOC(dai, VioConfig, vio_max_kfs))
    .def_readwrite("vio_min_frames_after_kf", &basalt::VioConfig::vio_min_frames_after_kf, DOC(dai, VioConfig, vio_min_frames_after_kf))
    .def_readwrite("vio_new_kf_keypoints_thresh", &basalt::VioConfig::vio_new_kf_keypoints_thresh, DOC(dai, VioConfig, vio_new_kf_keypoints_thresh))
    .def_readwrite("vio_debug", &basalt::VioConfig::vio_debug, DOC(dai, VioConfig, vio_debug))
    .def_readwrite("vio_extended_logging", &basalt::VioConfig::vio_extended_logging, DOC(dai, VioConfig, vio_extended_logging))
    .def_readwrite("vio_obs_std_dev", &basalt::VioConfig::vio_obs_std_dev, DOC(dai, VioConfig, vio_obs_std_dev))
    .def_readwrite("vio_obs_huber_thresh", &basalt::VioConfig::vio_obs_huber_thresh, DOC(dai, VioConfig, vio_obs_huber_thresh))
    .def_readwrite("vio_min_triangulation_dist", &basalt::VioConfig::vio_min_triangulation_dist, DOC(dai, VioConfig, vio_min_triangulation_dist))
    .def_readwrite("vio_max_iterations", &basalt::VioConfig::vio_max_iterations, DOC(dai, VioConfig, vio_max_iterations))
    .def_readwrite("vio_enforce_realtime", &basalt::VioConfig::vio_enforce_realtime, DOC(dai, VioConfig, vio_enforce_realtime))
    .def_readwrite("vio_use_lm", &basalt::VioConfig::vio_use_lm, DOC(dai, VioConfig, vio_use_lm))
    .def_readwrite("vio_lm_lambda_initial", &basalt::VioConfig::vio_lm_lambda_initial, DOC(dai, VioConfig, vio_lm_lambda_initial))
    .def_readwrite("vio_lm_lambda_min", &basalt::VioConfig::vio_lm_lambda_min, DOC(dai, VioConfig, vio_lm_lambda_min))
    .def_readwrite("vio_lm_lambda_max", &basalt::VioConfig::vio_lm_lambda_max, DOC(dai, VioConfig, vio_lm_lambda_max))
    .def_readwrite("vio_scale_jacobian", &basalt::VioConfig::vio_scale_jacobian, DOC(dai, VioConfig, vio_scale_jacobian))
    .def_readwrite("vio_init_pose_weight", &basalt::VioConfig::vio_init_pose_weight, DOC(dai, VioConfig, vio_init_pose_weight))
    .def_readwrite("vio_init_ba_weight", &basalt::VioConfig::vio_init_ba_weight, DOC(dai, VioConfig, vio_init_ba_weight))
    .def_readwrite("vio_init_bg_weight", &basalt::VioConfig::vio_init_bg_weight, DOC(dai, VioConfig, vio_init_bg_weight))
    .def_readwrite("vio_marg_lost_landmarks", &basalt::VioConfig::vio_marg_lost_landmarks, DOC(dai, VioConfig, vio_marg_lost_landmarks))
    .def_readwrite("vio_fix_long_term_keyframes", &basalt::VioConfig::vio_fix_long_term_keyframes, DOC(dai, VioConfig, vio_fix_long_term_keyframes))
    .def_readwrite("vio_kf_marg_feature_ratio", &basalt::VioConfig::vio_kf_marg_feature_ratio, DOC(dai, VioConfig, vio_kf_marg_feature_ratio))
    .def_readwrite("vio_kf_marg_criteria", &basalt::VioConfig::vio_kf_marg_criteria, DOC(dai, VioConfig, vio_kf_marg_criteria))
    .def_readwrite("mapper_obs_std_dev", &basalt::VioConfig::mapper_obs_std_dev, DOC(dai, VioConfig, mapper_obs_std_dev))
    .def_readwrite("mapper_obs_huber_thresh", &basalt::VioConfig::mapper_obs_huber_thresh, DOC(dai, VioConfig, mapper_obs_huber_thresh))
    .def_readwrite("mapper_detection_num_points", &basalt::VioConfig::mapper_detection_num_points, DOC(dai, VioConfig, mapper_detection_num_points))
    .def_readwrite("mapper_num_frames_to_match", &basalt::VioConfig::mapper_num_frames_to_match, DOC(dai, VioConfig, mapper_num_frames_to_match))
    .def_readwrite("mapper_frames_to_match_threshold", &basalt::VioConfig::mapper_frames_to_match_threshold, DOC(dai, VioConfig, mapper_frames_to_match_threshold))
    .def_readwrite("mapper_min_matches", &basalt::VioConfig::mapper_min_matches, DOC(dai, VioConfig, mapper_min_matches))
    .def_readwrite("mapper_ransac_threshold", &basalt::VioConfig::mapper_ransac_threshold, DOC(dai, VioConfig, mapper_ransac_threshold))
    .def_readwrite("mapper_min_track_length", &basalt::VioConfig::mapper_min_track_length, DOC(dai, VioConfig, mapper_min_track_length))
    .def_readwrite("mapper_max_hamming_distance", &basalt::VioConfig::mapper_max_hamming_distance, DOC(dai, VioConfig, mapper_max_hamming_distance))
    .def_readwrite("mapper_second_best_test_ratio", &basalt::VioConfig::mapper_second_best_test_ratio, DOC(dai, VioConfig, mapper_second_best_test_ratio))
    .def_readwrite("mapper_bow_num_bits", &basalt::VioConfig::mapper_bow_num_bits, DOC(dai, VioConfig, mapper_bow_num_bits))
    .def_readwrite("mapper_min_triangulation_dist", &basalt::VioConfig::mapper_min_triangulation_dist, DOC(dai, VioConfig, mapper_min_triangulation_dist))
    .def_readwrite("mapper_no_factor_weights", &basalt::VioConfig::mapper_no_factor_weights, DOC(dai, VioConfig, mapper_no_factor_weights))
    .def_readwrite("mapper_use_factors", &basalt::VioConfig::mapper_use_factors, DOC(dai, VioConfig, mapper_use_factors))
    .def_readwrite("mapper_use_lm", &basalt::VioConfig::mapper_use_lm, DOC(dai, VioConfig, mapper_use_lm))
    .def_readwrite("mapper_lm_lambda_min", &basalt::VioConfig::mapper_lm_lambda_min, DOC(dai, VioConfig, mapper_lm_lambda_min))
    .def_readwrite("mapper_lm_lambda_max", &basalt::VioConfig::mapper_lm_lambda_max, DOC(dai, VioConfig, mapper_lm_lambda_max));
    ///////////////////////////////////////////////////////////////////////
}