#include <memory>

#include "DatatypeBindings.hpp"
#include "pipeline/CommonBindings.hpp"

// depthai
#include "depthai/pipeline/datatype/TofFusionConfig.hpp"

// pybind
#include <pybind11/chrono.h>
#include <pybind11/numpy.h>

void bind_toffusionconfig(pybind11::module& m, void* pCallstack) {
    using namespace dai;

    py::class_<TofFusionConfig, Py<TofFusionConfig>, Buffer, std::shared_ptr<TofFusionConfig>> tofFusionConfig(
        m, "TofFusionConfig", DOC(dai, TofFusionConfig));

    ///////////////////////////////////////////////////////////////////////
    // Call the rest of the type defines, then perform the actual bindings
    Callstack* callstack = (Callstack*)pCallstack;
    auto cb = callstack->top();
    callstack->pop();
    cb(m, pCallstack);
    // Actual bindings
    ///////////////////////////////////////////////////////////////////////

    tofFusionConfig.def(py::init<>())
        .def("__repr__", &TofFusionConfig::str)
        .def_readwrite("tof_base_weight", &TofFusionConfig::tof_base_weight)
        .def_readwrite("tof_conf_power", &TofFusionConfig::tof_conf_power)
        .def_readwrite("tof_range_decay_start", &TofFusionConfig::tof_range_decay_start)
        .def_readwrite("tof_range_decay_rate", &TofFusionConfig::tof_range_decay_rate)
        .def_readwrite("neural_base_weight", &TofFusionConfig::neural_base_weight)
        .def_readwrite("neural_conf_power", &TofFusionConfig::neural_conf_power)
        .def_readwrite("texture_tof_boost", &TofFusionConfig::texture_tof_boost)
        .def_readwrite("texture_neural_boost", &TofFusionConfig::texture_neural_boost)
        .def_readwrite("agree_thresh_rel", &TofFusionConfig::agree_thresh_rel)
        .def_readwrite("agree_tof_bonus", &TofFusionConfig::agree_tof_bonus)
        .def_readwrite("disagree_neural_penalty", &TofFusionConfig::disagree_neural_penalty)
        .def_readwrite("neural_depth_scale", &TofFusionConfig::neural_depth_scale)
        .def_readwrite("tof_preproj_bias_m", &TofFusionConfig::tof_preproj_bias_m)
        .def_readwrite("tof_preproj_scale", &TofFusionConfig::tof_preproj_scale)
        .def_readwrite("tof_preproj_upscale", &TofFusionConfig::tof_preproj_upscale);
}
