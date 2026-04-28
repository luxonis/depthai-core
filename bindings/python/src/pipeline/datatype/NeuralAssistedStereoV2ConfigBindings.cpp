#include <memory>

#include "DatatypeBindings.hpp"
#include "pipeline/CommonBindings.hpp"

#include "depthai/pipeline/datatype/NeuralAssistedStereoV2Config.hpp"

#include <pybind11/chrono.h>
#include <pybind11/numpy.h>

void bind_neuralassistedstereov2config(pybind11::module& m, void* pCallstack) {
    using namespace dai;

    py::class_<NeuralAssistedStereoV2Config, Py<NeuralAssistedStereoV2Config>, Buffer, std::shared_ptr<NeuralAssistedStereoV2Config>> cfg(
        m, "NeuralAssistedStereoV2Config");
    py::class_<NeuralAssistedStereoV2Config::AlgorithmControl> algorithmControl(cfg, "AlgorithmControl");

    py::enum_<NeuralAssistedStereoV2Config::CostMethod>(cfg, "CostMethod")
        .value("SAD", NeuralAssistedStereoV2Config::CostMethod::SAD)
        .value("ZNCC", NeuralAssistedStereoV2Config::CostMethod::ZNCC)
        .export_values();

    py::enum_<NeuralAssistedStereoV2Config::PrefilterMethod>(cfg, "PrefilterMethod")
        .value("NONE", NeuralAssistedStereoV2Config::PrefilterMethod::NONE)
        .value("SOBEL_X", NeuralAssistedStereoV2Config::PrefilterMethod::SOBEL_X)
        .value("BOX_MEAN_SUBTRACT", NeuralAssistedStereoV2Config::PrefilterMethod::BOX_MEAN_SUBTRACT)
        .value("GAUSSIAN_3x3", NeuralAssistedStereoV2Config::PrefilterMethod::GAUSSIAN_3x3)
        .export_values();

    Callstack* callstack = (Callstack*)pCallstack;
    auto cb = callstack->top();
    callstack->pop();
    cb(m, pCallstack);

    cfg.def(py::init<>())
        .def("invalidDisparity", &NeuralAssistedStereoV2Config::invalid_disparity)
        // Neural fusion
        .def_readwrite("kappa0", &NeuralAssistedStereoV2Config::kappa0)
        .def_readwrite("radiusScale", &NeuralAssistedStereoV2Config::radius_scale)
        .def_readwrite("radiusMin", &NeuralAssistedStereoV2Config::radius_min)
        .def_readwrite("radiusMax", &NeuralAssistedStereoV2Config::radius_max)
        // Matching
        .def_readwrite("costMethod", &NeuralAssistedStereoV2Config::cost_method)
        .def_readwrite("blockMatchRadius", &NeuralAssistedStereoV2Config::block_match_radius)
        .def_readwrite("subpixelBits", &NeuralAssistedStereoV2Config::subpixel_bits)
        .def_readwrite("minDisp", &NeuralAssistedStereoV2Config::min_disp)
        .def_readwrite("maxDisparity", &NeuralAssistedStereoV2Config::max_disparity)
        // Prefilter
        .def_readwrite("prefilterMethod", &NeuralAssistedStereoV2Config::prefilter_method)
        .def_readwrite("prefilterSigmaSpatial", &NeuralAssistedStereoV2Config::prefilter_sigma_spatial)
        .def_readwrite("prefilterSigmaRange", &NeuralAssistedStereoV2Config::prefilter_sigma_range)
        // Post-processing
        .def_readwrite("lrCheck", &NeuralAssistedStereoV2Config::lr_check)
        .def_readwrite("lrCheckFast", &NeuralAssistedStereoV2Config::lr_check_fast)
        .def_readwrite("medianSize", &NeuralAssistedStereoV2Config::median_size)
        .def_readwrite("confidenceThreshold", &NeuralAssistedStereoV2Config::confidence_threshold)
        .def_readwrite("speckleMaxSize", &NeuralAssistedStereoV2Config::speckle_max_size)
        .def_readwrite("speckleMaxDiff", &NeuralAssistedStereoV2Config::speckle_max_diff)
        .def_readwrite("holeFillRadius", &NeuralAssistedStereoV2Config::hole_fill_radius)
        .def_readwrite("holeFillSigmaSpatial", &NeuralAssistedStereoV2Config::hole_fill_sigma_spatial)
        .def_readwrite("holeFillSigmaRange", &NeuralAssistedStereoV2Config::hole_fill_sigma_range)
        .def_readwrite("temporalAlpha", &NeuralAssistedStereoV2Config::temporal_alpha)
        .def_readwrite("temporalDelta", &NeuralAssistedStereoV2Config::temporal_delta)
        .def_readwrite("temporalPersistencyMode", &NeuralAssistedStereoV2Config::temporal_persistency_mode)
        .def_readwrite("secondPeakThreshold", &NeuralAssistedStereoV2Config::second_peak_threshold)
        .def_readwrite("secondPeakMinGap", &NeuralAssistedStereoV2Config::second_peak_min_gap)
        // Output
        .def_readwrite("useFp16", &NeuralAssistedStereoV2Config::use_fp16)
        .def_readwrite("algorithmControl", &NeuralAssistedStereoV2Config::algorithmControl)
        // Debug
        .def_readwrite("debugZnccPlotX", &NeuralAssistedStereoV2Config::debug_zncc_plot_x)
        .def_readwrite("debugZnccPlotY", &NeuralAssistedStereoV2Config::debug_zncc_plot_y);

    algorithmControl.def(py::init<>())
        .def_readwrite("depthUnit", &NeuralAssistedStereoV2Config::AlgorithmControl::depthUnit)
        .def_readwrite("customDepthUnitMultiplier", &NeuralAssistedStereoV2Config::AlgorithmControl::customDepthUnitMultiplier);
}
