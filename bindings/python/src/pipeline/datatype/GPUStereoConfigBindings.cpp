#include <memory>

#include "DatatypeBindings.hpp"
#include "depthai/pipeline/datatype/GPUStereoConfig.hpp"
#include "pipeline/CommonBindings.hpp"

void bind_gpustereoconfig(pybind11::module& m, void* pCallstack) {
    using namespace dai;

    py::class_<GPUStereoConfig, Py<GPUStereoConfig>, Buffer, std::shared_ptr<GPUStereoConfig>> gpuStereoConfig(m, "GPUStereoConfig", DOC(dai, GPUStereoConfig));
    py::enum_<GPUStereoConfig::ComputeMode> computeMode(gpuStereoConfig, "ComputeMode", DOC(dai, GPUStereoConfig, ComputeMode));

    Callstack* callstack = (Callstack*)pCallstack;
    auto cb = callstack->top();
    callstack->pop();
    cb(m, pCallstack);

    computeMode.value("GPU_CPU", GPUStereoConfig::ComputeMode::GPU_CPU).value("GPU_ONLY", GPUStereoConfig::ComputeMode::GPU_ONLY);

    gpuStereoConfig.def(py::init<>())
        .def("__repr__", &GPUStereoConfig::str)
        .def("setConfidenceThreshold", &GPUStereoConfig::setConfidenceThreshold, py::arg("threshold"), DOC(dai, GPUStereoConfig, setConfidenceThreshold))
        .def("getConfidenceThreshold", &GPUStereoConfig::getConfidenceThreshold, DOC(dai, GPUStereoConfig, getConfidenceThreshold))
        .def_readwrite("confidenceThreshold", &GPUStereoConfig::confidenceThreshold, DOC(dai, GPUStereoConfig, confidenceThreshold))
        .def("setComputeMode", &GPUStereoConfig::setComputeMode, py::arg("mode"), DOC(dai, GPUStereoConfig, setComputeMode))
        .def("getComputeMode", &GPUStereoConfig::getComputeMode, DOC(dai, GPUStereoConfig, getComputeMode))
        .def_readwrite("computeMode", &GPUStereoConfig::computeMode, DOC(dai, GPUStereoConfig, computeMode))
        .def("setTemporalFilter", &GPUStereoConfig::setTemporalFilter, py::arg("enable"), DOC(dai, GPUStereoConfig, setTemporalFilter))
        .def("getTemporalFilter", &GPUStereoConfig::getTemporalFilter, DOC(dai, GPUStereoConfig, getTemporalFilter))
        .def_readwrite("temporalFilter", &GPUStereoConfig::temporalFilter, DOC(dai, GPUStereoConfig, temporalFilter));
}
