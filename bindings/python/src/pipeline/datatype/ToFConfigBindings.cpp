#include <memory>
#include <unordered_map>

#include "DatatypeBindings.hpp"
#include "pipeline/CommonBindings.hpp"

// depthai
#include "depthai/pipeline/datatype/ToFConfig.hpp"

// pybind
#include <pybind11/chrono.h>
#include <pybind11/numpy.h>

// #include "spdlog/spdlog.h"

void bind_tofconfig(pybind11::module& m, void* pCallstack) {
    using namespace dai;

    py::class_<ToFConfig, Py<ToFConfig>, Buffer, std::shared_ptr<ToFConfig>> toFConfig(m, "ToFConfig", DOC(dai, ToFConfig));

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

    // Message
    toFConfig.def(py::init<>())
        .def("__repr__", &ToFConfig::str)
        // .def(py::init<std::shared_ptr<ToFConfig>>())

        .def_readwrite("median", &ToFConfig::median, DOC(dai, ToFConfig, median))
        .def_readwrite("enablePhaseShuffleTemporalFilter", &ToFConfig::enablePhaseShuffleTemporalFilter, DOC(dai, ToFConfig, enablePhaseShuffleTemporalFilter))
        .def_readwrite("enableBurstMode", &ToFConfig::enableBurstMode, DOC(dai, ToFConfig, enableBurstMode))
        .def_readwrite("enableDistortionCorrection", &ToFConfig::enableDistortionCorrection, DOC(dai, ToFConfig, enableDistortionCorrection))
        .def_readwrite("phaseUnwrappingLevel", &ToFConfig::phaseUnwrappingLevel, DOC(dai, ToFConfig, phaseUnwrappingLevel))
        .def_readwrite("enableFPPNCorrection", &ToFConfig::enableFPPNCorrection, DOC(dai, ToFConfig, enableFPPNCorrection))
        .def_readwrite("enableOpticalCorrection", &ToFConfig::enableOpticalCorrection, DOC(dai, ToFConfig, enableOpticalCorrection))
        .def_readwrite("enableTemperatureCorrection", &ToFConfig::enableTemperatureCorrection, DOC(dai, ToFConfig, enableTemperatureCorrection))
        .def_readwrite("enableWiggleCorrection", &ToFConfig::enableWiggleCorrection, DOC(dai, ToFConfig, enableWiggleCorrection))
        .def_readwrite("enablePhaseUnwrapping", &ToFConfig::enablePhaseUnwrapping, DOC(dai, ToFConfig, enablePhaseUnwrapping))
        .def_readwrite("phaseUnwrapErrorThreshold", &ToFConfig::phaseUnwrapErrorThreshold, DOC(dai, ToFConfig, phaseUnwrapErrorThreshold))

        .def("setMedianFilter", &ToFConfig::setMedianFilter, DOC(dai, ToFConfig, setMedianFilter))

        // .def("set", &ToFConfig::set, py::arg("config"), DOC(dai, ToFConfig, set))
        // .def("get", &ToFConfig::get, DOC(dai, ToFConfig, get))
        ;

    // add aliases
    // m.attr("ToFConfig").attr("DepthParams") = m.attr("ToFConfig").attr("DepthParams");
}
