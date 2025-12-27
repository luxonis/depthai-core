#include <pybind11/eval.h>

#include "Common.hpp"
#include "NodeBindings.hpp"
#include "depthai/basalt/BasaltVIO.hpp"
#include "depthai/pipeline/ThreadedHostNode.hpp"

extern py::handle daiNodeModule;

void bind_basaltnode(pybind11::module& m, void* pCallstack) {
    using namespace dai;
    using namespace dai::node;

    // declare upfront
    auto basaltNode = ADD_NODE_DERIVED(BasaltVIO, ThreadedHostNode);
    py::class_<BasaltVIO::VioConfig> vioConfig(m, "VioConfig");
    ///////////////////////////////////////////////////////////////////////
    ///////////////////////////////////////////////////////////////////////
    ///////////////////////////////////////////////////////////////////////
    // Call the rest of the type defines, then perform the actual bindings
    Callstack* callstack = (Callstack*)pCallstack;
    auto cb = callstack->top();
    callstack->pop();
    cb(m, pCallstack);
    // Actual bindings
    ///////////////////////////////////////////////////////////////////////
    ///////////////////////////////////////////////////////////////////////
    ///////////////////////////////////////////////////////////////////////
    // BasaltVIO Node
    basaltNode.def_property_readonly(
                  "left", [](BasaltVIO& node) { return &node.left; }, py::return_value_policy::reference_internal)
        .def_property_readonly(
            "right", [](BasaltVIO& node) { return &node.right; }, py::return_value_policy::reference_internal)
        .def_readonly("imu", &BasaltVIO::imu, DOC(dai, node, BasaltVIO, imu))
        .def_readonly("transform", &BasaltVIO::transform, DOC(dai, node, BasaltVIO, transform))
        .def_readonly("passthrough", &BasaltVIO::passthrough, DOC(dai, node, BasaltVIO, passthrough))
        .def("setImuUpdateRate", &BasaltVIO::setImuUpdateRate, py::arg("rate"), DOC(dai, node, BasaltVIO, setImuUpdateRate))
        .def("setConfigPath", &BasaltVIO::setConfigPath, py::arg("path"), DOC(dai, node, BasaltVIO, setConfigPath))
        .def("setVioConfig", &BasaltVIO::setConfig, py::arg("config"), DOC(dai, node, BasaltVIO, setConfig))
        .def("setAccelBias", &BasaltVIO::setAccelBias, py::arg("bias"))
        .def("setGyroBias", &BasaltVIO::setGyroBias, py::arg("bias"))
        .def("setAccelNoiseStd", &BasaltVIO::setAccelNoiseStd, py::arg("noise"))
        .def("setGyroNoiseStd", &BasaltVIO::setGyroNoiseStd, py::arg("noise"))
        .def("runSyncOnHost", &BasaltVIO::runSyncOnHost, py::arg("runOnHost"), DOC(dai, node, BasaltVIO, runSyncOnHost))
        .def("setImuExtrinsics", &BasaltVIO::setImuExtrinsics, py::arg("imuExtr"), DOC(dai, node, BasaltVIO, setImuExtrinsics))
        .def("setLocalTransform", &BasaltVIO::setLocalTransform, py::arg("transform"), DOC(dai, node, BasaltVIO, setLocalTransform));
    // VioConfig
    vioConfig.def(py::init<>())
        .def_readwrite("optical_flow_type", &BasaltVIO::VioConfig::optical_flow_type)
        .def_readwrite("optical_flow_detection_grid_size", &BasaltVIO::VioConfig::optical_flow_detection_grid_size)
        .def_readwrite("optical_flow_detection_num_points_cell", &BasaltVIO::VioConfig::optical_flow_detection_num_points_cell)
        .def_readwrite("optical_flow_detection_min_threshold", &BasaltVIO::VioConfig::optical_flow_detection_min_threshold)
        .def_readwrite("optical_flow_detection_max_threshold", &BasaltVIO::VioConfig::optical_flow_detection_max_threshold)
        .def_readwrite("optical_flow_detection_nonoverlap", &BasaltVIO::VioConfig::optical_flow_detection_nonoverlap)
        .def_readwrite("optical_flow_max_recovered_dist2", &BasaltVIO::VioConfig::optical_flow_max_recovered_dist2)
        .def_readwrite("optical_flow_pattern", &BasaltVIO::VioConfig::optical_flow_pattern)
        .def_readwrite("optical_flow_max_iterations", &BasaltVIO::VioConfig::optical_flow_max_iterations)
        .def_readwrite("optical_flow_epipolar_error", &BasaltVIO::VioConfig::optical_flow_epipolar_error)
        .def_readwrite("optical_flow_levels", &BasaltVIO::VioConfig::optical_flow_levels)
        .def_readwrite("optical_flow_skip_frames", &BasaltVIO::VioConfig::optical_flow_skip_frames)
        .def_readwrite("optical_flow_matching_guess_type", &BasaltVIO::VioConfig::optical_flow_matching_guess_type)
        .def_readwrite("optical_flow_matching_default_depth", &BasaltVIO::VioConfig::optical_flow_matching_default_depth)
        .def_readwrite("optical_flow_image_safe_radius", &BasaltVIO::VioConfig::optical_flow_image_safe_radius)
        .def_readwrite("optical_flow_recall_enable", &BasaltVIO::VioConfig::optical_flow_recall_enable)
        .def_readwrite("optical_flow_recall_all_cams", &BasaltVIO::VioConfig::optical_flow_recall_all_cams)
        .def_readwrite("optical_flow_recall_num_points_cell", &BasaltVIO::VioConfig::optical_flow_recall_num_points_cell)
        .def_readwrite("optical_flow_recall_over_tracking", &BasaltVIO::VioConfig::optical_flow_recall_over_tracking)
        .def_readwrite("optical_flow_recall_update_patch_viewpoint", &BasaltVIO::VioConfig::optical_flow_recall_update_patch_viewpoint)
        .def_readwrite("optical_flow_recall_max_patch_dist", &BasaltVIO::VioConfig::optical_flow_recall_max_patch_dist)
        .def_readwrite("optical_flow_recall_max_patch_norms", &BasaltVIO::VioConfig::optical_flow_recall_max_patch_norms)
        .def_readwrite("vio_linearization_type", &BasaltVIO::VioConfig::vio_linearization_type)
        .def_readwrite("vio_sqrt_marg", &BasaltVIO::VioConfig::vio_sqrt_marg)
        .def_readwrite("vio_max_states", &BasaltVIO::VioConfig::vio_max_states)
        .def_readwrite("vio_max_kfs", &BasaltVIO::VioConfig::vio_max_kfs)
        .def_readwrite("vio_min_frames_after_kf", &BasaltVIO::VioConfig::vio_min_frames_after_kf)
        .def_readwrite("vio_new_kf_keypoints_thresh", &BasaltVIO::VioConfig::vio_new_kf_keypoints_thresh)
        .def_readwrite("vio_debug", &BasaltVIO::VioConfig::vio_debug)
        .def_readwrite("vio_extended_logging", &BasaltVIO::VioConfig::vio_extended_logging)
        .def_readwrite("vio_obs_std_dev", &BasaltVIO::VioConfig::vio_obs_std_dev)
        .def_readwrite("vio_obs_huber_thresh", &BasaltVIO::VioConfig::vio_obs_huber_thresh)
        .def_readwrite("vio_min_triangulation_dist", &BasaltVIO::VioConfig::vio_min_triangulation_dist)
        .def_readwrite("vio_max_iterations", &BasaltVIO::VioConfig::vio_max_iterations)
        .def_readwrite("vio_enforce_realtime", &BasaltVIO::VioConfig::vio_enforce_realtime)
        .def_readwrite("vio_use_lm", &BasaltVIO::VioConfig::vio_use_lm)
        .def_readwrite("vio_lm_lambda_initial", &BasaltVIO::VioConfig::vio_lm_lambda_initial)
        .def_readwrite("vio_lm_lambda_min", &BasaltVIO::VioConfig::vio_lm_lambda_min)
        .def_readwrite("vio_lm_lambda_max", &BasaltVIO::VioConfig::vio_lm_lambda_max)
        .def_readwrite("vio_scale_jacobian", &BasaltVIO::VioConfig::vio_scale_jacobian)
        .def_readwrite("vio_init_pose_weight", &BasaltVIO::VioConfig::vio_init_pose_weight)
        .def_readwrite("vio_init_ba_weight", &BasaltVIO::VioConfig::vio_init_ba_weight)
        .def_readwrite("vio_init_bg_weight", &BasaltVIO::VioConfig::vio_init_bg_weight)
        .def_readwrite("vio_marg_lost_landmarks", &BasaltVIO::VioConfig::vio_marg_lost_landmarks)
        .def_readwrite("vio_fix_long_term_keyframes", &BasaltVIO::VioConfig::vio_fix_long_term_keyframes)
        .def_readwrite("vio_kf_marg_feature_ratio", &BasaltVIO::VioConfig::vio_kf_marg_feature_ratio)
        .def_readwrite("vio_kf_marg_criteria", &BasaltVIO::VioConfig::vio_kf_marg_criteria)
        .def_readwrite("mapper_obs_std_dev", &BasaltVIO::VioConfig::mapper_obs_std_dev)
        .def_readwrite("mapper_obs_huber_thresh", &BasaltVIO::VioConfig::mapper_obs_huber_thresh)
        .def_readwrite("mapper_detection_num_points", &BasaltVIO::VioConfig::mapper_detection_num_points)
        .def_readwrite("mapper_num_frames_to_match", &BasaltVIO::VioConfig::mapper_num_frames_to_match)
        .def_readwrite("mapper_frames_to_match_threshold", &BasaltVIO::VioConfig::mapper_frames_to_match_threshold)
        .def_readwrite("mapper_min_matches", &BasaltVIO::VioConfig::mapper_min_matches)
        .def_readwrite("mapper_ransac_threshold", &BasaltVIO::VioConfig::mapper_ransac_threshold)
        .def_readwrite("mapper_min_track_length", &BasaltVIO::VioConfig::mapper_min_track_length)
        .def_readwrite("mapper_max_hamming_distance", &BasaltVIO::VioConfig::mapper_max_hamming_distance)
        .def_readwrite("mapper_second_best_test_ratio", &BasaltVIO::VioConfig::mapper_second_best_test_ratio)
        .def_readwrite("mapper_bow_num_bits", &BasaltVIO::VioConfig::mapper_bow_num_bits)
        .def_readwrite("mapper_min_triangulation_dist", &BasaltVIO::VioConfig::mapper_min_triangulation_dist)
        .def_readwrite("mapper_no_factor_weights", &BasaltVIO::VioConfig::mapper_no_factor_weights)
        .def_readwrite("mapper_use_factors", &BasaltVIO::VioConfig::mapper_use_factors)
        .def_readwrite("mapper_use_lm", &BasaltVIO::VioConfig::mapper_use_lm)
        .def_readwrite("mapper_lm_lambda_min", &BasaltVIO::VioConfig::mapper_lm_lambda_min)
        .def_readwrite("mapper_lm_lambda_max", &BasaltVIO::VioConfig::mapper_lm_lambda_max);
    ///////////////////////////////////////////////////////////////////////
}
