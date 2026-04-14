#include <memory>
#include <unordered_map>

#include "DatatypeBindings.hpp"
#include "pipeline/CommonBindings.hpp"

#include "depthai/pipeline/datatype/GPUStereoConfig.hpp"

#include <pybind11/chrono.h>
#include <pybind11/numpy.h>

void bind_gpustereoconfig(pybind11::module& m, void* pCallstack) {
    using namespace dai;

    py::class_<GPUStereoConfig, Py<GPUStereoConfig>, Buffer, std::shared_ptr<GPUStereoConfig>> gpuStereoConfig(
        m, "GPUStereoConfig", DOC(dai, GPUStereoConfig));
    py::class_<GPUStereoConfig::AlgorithmControl> algorithmControl(gpuStereoConfig, "AlgorithmControl", DOC(dai, GPUStereoConfig, AlgorithmControl));

    py::enum_<GPUStereoConfig::DownsampleMethod>(gpuStereoConfig, "DownsampleMethod", DOC(dai, GPUStereoConfig, DownsampleMethod))
        .value("BOX_FILTER", GPUStereoConfig::DownsampleMethod::BOX_FILTER)
        .value("BILINEAR", GPUStereoConfig::DownsampleMethod::BILINEAR)
        .value("GAUSSIAN_5x5", GPUStereoConfig::DownsampleMethod::GAUSSIAN_5x5)
        .value("GAUSSIAN_3x3", GPUStereoConfig::DownsampleMethod::GAUSSIAN_3x3)
        .value("NEAREST", GPUStereoConfig::DownsampleMethod::NEAREST)
        .export_values();
    py::enum_<GPUStereoConfig::PrefilterMethod>(gpuStereoConfig, "PrefilterMethod", DOC(dai, GPUStereoConfig, PrefilterMethod))
        .value("NONE", GPUStereoConfig::PrefilterMethod::NONE)
        .value("SOBEL_X", GPUStereoConfig::PrefilterMethod::SOBEL_X)
        .value("BOX_MEAN_SUBTRACT", GPUStereoConfig::PrefilterMethod::BOX_MEAN_SUBTRACT)
        .value("GAUSSIAN_3x3", GPUStereoConfig::PrefilterMethod::GAUSSIAN_3x3)
        .value("BILATERAL_PREFILTER", GPUStereoConfig::PrefilterMethod::BILATERAL_PREFILTER)
        .export_values();
    py::enum_<GPUStereoConfig::CostVolumeAggregation>(gpuStereoConfig, "CostVolumeAggregation", DOC(dai, GPUStereoConfig, CostVolumeAggregation))
        .value("WTA", GPUStereoConfig::CostVolumeAggregation::WTA)
        .value("BOX", GPUStereoConfig::CostVolumeAggregation::BOX)
        .value("BILATERAL", GPUStereoConfig::CostVolumeAggregation::BILATERAL)
        .export_values();
    py::enum_<GPUStereoConfig::CostMethod>(gpuStereoConfig, "CostMethod", DOC(dai, GPUStereoConfig, CostMethod))
        .value("SAD", GPUStereoConfig::CostMethod::SAD)
        .value("CENSUS", GPUStereoConfig::CostMethod::CENSUS)
        .value("GRADIENT", GPUStereoConfig::CostMethod::GRADIENT)
        .value("ZNCC", GPUStereoConfig::CostMethod::ZNCC)
        .value("RANK", GPUStereoConfig::CostMethod::RANK)
        .export_values();
    py::enum_<GPUStereoConfig::PathAggregation>(gpuStereoConfig, "PathAggregation", DOC(dai, GPUStereoConfig, PathAggregation))
        .value("NONE", GPUStereoConfig::PathAggregation::NONE)
        .value("SGM_2", GPUStereoConfig::PathAggregation::SGM_2)
        .value("SGM_4", GPUStereoConfig::PathAggregation::SGM_4)
        .value("SGM_8", GPUStereoConfig::PathAggregation::SGM_8)
        .value("SGM_MGM_INPLACE", GPUStereoConfig::PathAggregation::SGM_MGM_INPLACE)
        .export_values();
    py::enum_<GPUStereoConfig::SpeckleFilterBackend>(gpuStereoConfig, "SpeckleFilterBackend", py::doc("When speckleMaxSize > 0: OPENCL runs speckle on GPU; OPENCV uses cv::filterSpeckles on the host after stereo (same family as StereoDepth)."))
        .value("OPENCL", GPUStereoConfig::SpeckleFilterBackend::OPENCL)
        .value("OPENCV", GPUStereoConfig::SpeckleFilterBackend::OPENCV)
        .export_values();

    Callstack* callstack = (Callstack*)pCallstack;
    auto cb = callstack->top();
    callstack->pop();
    cb(m, pCallstack);

    gpuStereoConfig.def(py::init<>())
        .def("__repr__", &GPUStereoConfig::str)
        .def("invalidDisparity", &GPUStereoConfig::invalid_disparity, DOC(dai, GPUStereoConfig, invalid_disparity))
        .def_readwrite("maxDisparity", &GPUStereoConfig::max_disparity, DOC(dai, GPUStereoConfig, max_disparity))
        .def_readwrite("numPyramidLevels", &GPUStereoConfig::num_pyramid_levels, DOC(dai, GPUStereoConfig, num_pyramid_levels))
        .def_readwrite("downsampleMethod", &GPUStereoConfig::downsample_method, DOC(dai, GPUStereoConfig, downsample_method))
        .def_readwrite("prefilterMethod", &GPUStereoConfig::prefilter_method, DOC(dai, GPUStereoConfig, prefilter_method))
        .def_readwrite("blockMatchRadius", &GPUStereoConfig::block_match_radius, DOC(dai, GPUStereoConfig, block_match_radius))
        .def_readwrite(
            "adaptiveSupportRangeSigma", &GPUStereoConfig::adaptive_support_range_sigma, DOC(dai, GPUStereoConfig, adaptive_support_range_sigma))
        .def_readwrite(
            "prefilterBilateralSigmaSpatial", &GPUStereoConfig::prefilter_bilateral_sigma_spatial, DOC(dai, GPUStereoConfig, prefilter_bilateral_sigma_spatial))
        .def_readwrite(
            "prefilterBilateralSigmaRange", &GPUStereoConfig::prefilter_bilateral_sigma_range, DOC(dai, GPUStereoConfig, prefilter_bilateral_sigma_range))
        .def_readwrite("refinementRadius", &GPUStereoConfig::refinement_radius, DOC(dai, GPUStereoConfig, refinement_radius))
        .def_readwrite("refinementRadiusFull", &GPUStereoConfig::refinement_radius_full, DOC(dai, GPUStereoConfig, refinement_radius_full))
        .def_readwrite("subpixelBits", &GPUStereoConfig::subpixel_bits, DOC(dai, GPUStereoConfig, subpixel_bits))
        .def_readwrite("lrCheck", &GPUStereoConfig::lr_check, DOC(dai, GPUStereoConfig, lr_check))
        .def_readwrite("lrCheckFast", &GPUStereoConfig::lr_check_fast, DOC(dai, GPUStereoConfig, lr_check_fast))
        .def_readwrite("medianSize", &GPUStereoConfig::median_size, DOC(dai, GPUStereoConfig, median_size))
        .def_readwrite("minDisp", &GPUStereoConfig::min_disp, DOC(dai, GPUStereoConfig, min_disp))
        .def_readwrite("confidenceThreshold", &GPUStereoConfig::confidence_threshold, DOC(dai, GPUStereoConfig, confidence_threshold))
        .def_readwrite("useCostVolume", &GPUStereoConfig::use_cost_volume, DOC(dai, GPUStereoConfig, use_cost_volume))
        .def_readwrite("costVolumeAggregation", &GPUStereoConfig::cost_volume_aggregation, DOC(dai, GPUStereoConfig, cost_volume_aggregation))
        .def_readwrite("boxAggregationRadius", &GPUStereoConfig::box_aggregation_radius, DOC(dai, GPUStereoConfig, box_aggregation_radius))
        .def_readwrite("bilateralSpatialSigma", &GPUStereoConfig::bilateral_spatial_sigma, DOC(dai, GPUStereoConfig, bilateral_spatial_sigma))
        .def_readwrite("bilateralRangeSigma", &GPUStereoConfig::bilateral_range_sigma, DOC(dai, GPUStereoConfig, bilateral_range_sigma))
        .def_readwrite(
            "bilateralAggregationRadius", &GPUStereoConfig::bilateral_aggregation_radius, DOC(dai, GPUStereoConfig, bilateral_aggregation_radius))
        .def_readwrite("costMethod", &GPUStereoConfig::cost_method, DOC(dai, GPUStereoConfig, cost_method))
        .def_readwrite("pathAggregation", &GPUStereoConfig::path_aggregation, DOC(dai, GPUStereoConfig, path_aggregation))
        .def_readwrite("sgmP1", &GPUStereoConfig::sgm_p1, DOC(dai, GPUStereoConfig, sgm_p1))
        .def_readwrite("sgmP2", &GPUStereoConfig::sgm_p2, DOC(dai, GPUStereoConfig, sgm_p2))
        .def_readwrite("sgmAdaptiveP2", &GPUStereoConfig::sgm_adaptive_p2, DOC(dai, GPUStereoConfig, sgm_adaptive_p2))
        .def_readwrite("useFp16", &GPUStereoConfig::use_fp16, DOC(dai, GPUStereoConfig, use_fp16))
        .def_readwrite("useQcomAcceleratedOps", &GPUStereoConfig::use_qcom_accelerated_ops, DOC(dai, GPUStereoConfig, use_qcom_accelerated_ops))
        .def_readwrite(
            "debugPyramidLevel",
            &GPUStereoConfig::debug_pyramid_level,
            py::doc("Pyramid level streamed on debugPyramid output when >= 0; -1 disables."))
        .def_readwrite(
            "debugPyramidDisparityLevel",
            &GPUStereoConfig::debug_pyramid_disparity_level,
            py::doc("Pyramid level streamed on debugPyramidDisparity (RAW16); -1 disables."))
        .def_readwrite(
            "debugZnccPlotX",
            &GPUStereoConfig::debug_zncc_plot_x,
            py::doc("Pyramid pixel X for ZNCC cost curve on debugZnccCurve; -1 disables."))
        .def_readwrite(
            "debugZnccPlotY",
            &GPUStereoConfig::debug_zncc_plot_y,
            py::doc("Pyramid pixel Y for ZNCC cost curve on debugZnccCurve; -1 disables."))
        .def_readwrite("secondPeakThreshold", &GPUStereoConfig::second_peak_threshold, DOC(dai, GPUStereoConfig, second_peak_threshold))
        .def_readwrite(
            "secondPeakMinDisparityGap", &GPUStereoConfig::second_peak_min_disparity_gap, DOC(dai, GPUStereoConfig, second_peak_min_disparity_gap))
        .def_readwrite("censusRadiusX", &GPUStereoConfig::census_radius_x, DOC(dai, GPUStereoConfig, census_radius_x))
        .def_readwrite("censusRadiusY", &GPUStereoConfig::census_radius_y, DOC(dai, GPUStereoConfig, census_radius_y))
        .def_readwrite("speckleMaxSize", &GPUStereoConfig::speckle_max_size, DOC(dai, GPUStereoConfig, speckle_max_size))
        .def_readwrite("speckleMaxDiff", &GPUStereoConfig::speckle_max_diff, DOC(dai, GPUStereoConfig, speckle_max_diff))
        .def_readwrite(
            "speckleFilterBackend",
            &GPUStereoConfig::speckle_filter_backend,
            py::doc("OPENCL (default) or OPENCV; only used when speckleMaxSize > 0."))
        .def_readwrite("textureFilterRadius", &GPUStereoConfig::texture_filter_radius, DOC(dai, GPUStereoConfig, texture_filter_radius))
        .def_readwrite("textureThreshold", &GPUStereoConfig::texture_threshold, DOC(dai, GPUStereoConfig, texture_threshold))
        .def_readwrite("featureMaskEdgeThresh", &GPUStereoConfig::feature_mask_edge_thresh, DOC(dai, GPUStereoConfig, feature_mask_edge_thresh))
        .def_readwrite("featureMaskCornerThresh", &GPUStereoConfig::feature_mask_corner_thresh, DOC(dai, GPUStereoConfig, feature_mask_corner_thresh))
        .def_readwrite("featureMaskMorphRadius", &GPUStereoConfig::feature_mask_morph_radius, DOC(dai, GPUStereoConfig, feature_mask_morph_radius))
        .def_readwrite("edgeAwareRadius", &GPUStereoConfig::edge_aware_radius, DOC(dai, GPUStereoConfig, edge_aware_radius))
        .def_readwrite("edgeAwareEps", &GPUStereoConfig::edge_aware_eps, DOC(dai, GPUStereoConfig, edge_aware_eps))
        .def_readwrite("holeFillRadius", &GPUStereoConfig::hole_fill_radius, DOC(dai, GPUStereoConfig, hole_fill_radius))
        .def_readwrite("holeFillSigmaSpatial", &GPUStereoConfig::hole_fill_sigma_spatial, DOC(dai, GPUStereoConfig, hole_fill_sigma_spatial))
        .def_readwrite("holeFillSigmaRange", &GPUStereoConfig::hole_fill_sigma_range, DOC(dai, GPUStereoConfig, hole_fill_sigma_range))
        .def_readwrite("temporalAlpha", &GPUStereoConfig::temporal_alpha, DOC(dai, GPUStereoConfig, temporal_alpha))
        .def_readwrite("temporalDelta", &GPUStereoConfig::temporal_delta, DOC(dai, GPUStereoConfig, temporal_delta))
        .def_readwrite("temporalPersistencyMode", &GPUStereoConfig::temporal_persistency_mode, DOC(dai, GPUStereoConfig, temporal_persistency_mode))
        .def_readwrite("regionRefine", &GPUStereoConfig::region_refine, DOC(dai, GPUStereoConfig, region_refine))
        .def_readwrite("regionRefineCellSize", &GPUStereoConfig::region_refine_cell_size, DOC(dai, GPUStereoConfig, region_refine_cell_size))
        .def_readwrite(
            "regionRefinePlaneResidualThresh",
            &GPUStereoConfig::region_refine_plane_residual_thresh,
            DOC(dai, GPUStereoConfig, region_refine_plane_residual_thresh))
        .def_readwrite("algorithmControl", &GPUStereoConfig::algorithmControl, DOC(dai, GPUStereoConfig, algorithmControl));

    algorithmControl.def(py::init<>())
        .def_readwrite("depthUnit", &GPUStereoConfig::AlgorithmControl::depthUnit, DOC(dai, GPUStereoConfig, AlgorithmControl, depthUnit))
        .def_readwrite(
            "customDepthUnitMultiplier",
            &GPUStereoConfig::AlgorithmControl::customDepthUnitMultiplier,
            DOC(dai, GPUStereoConfig, AlgorithmControl, customDepthUnitMultiplier));
}
