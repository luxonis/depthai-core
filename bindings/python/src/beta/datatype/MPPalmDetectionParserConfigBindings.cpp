#include "DatatypeBindings.hpp"
#include "depthai/beta/datatype/MPPalmDetectionParserConfig.hpp"
#include "pipeline/CommonBindings.hpp"

void bind_beta_mppalmdetectionparserconfig(pybind11::module& m, void* pCallstack) {
    using namespace dai;
    auto betaModule = m.def_submodule("beta", "Experimental APIs");
    py::class_<beta::MPPalmDetectionParserConfig, Py<beta::MPPalmDetectionParserConfig>, Buffer, std::shared_ptr<beta::MPPalmDetectionParserConfig>> config(
        betaModule, "MPPalmDetectionParserConfig", DOC(dai, beta, MPPalmDetectionParserConfig));

    Callstack* callstack = (Callstack*)pCallstack;
    auto cb = callstack->top();
    callstack->pop();
    cb(m, pCallstack);

    config.def(py::init<>())
        .def("__repr__", &beta::MPPalmDetectionParserConfig::str)
        .def_readwrite("confidenceThreshold", &beta::MPPalmDetectionParserConfig::confidenceThreshold)
        .def_readwrite("iouThreshold", &beta::MPPalmDetectionParserConfig::iouThreshold)
        .def_readwrite("maxDetections", &beta::MPPalmDetectionParserConfig::maxDetections)
        .def("setConfidenceThreshold",
             &beta::MPPalmDetectionParserConfig::setConfidenceThreshold,
             py::arg("threshold"),
             DOC(dai, beta, MPPalmDetectionParserConfig, setConfidenceThreshold))
        .def("getConfidenceThreshold",
             &beta::MPPalmDetectionParserConfig::getConfidenceThreshold,
             DOC(dai, beta, MPPalmDetectionParserConfig, getConfidenceThreshold))
        .def("setIouThreshold",
             &beta::MPPalmDetectionParserConfig::setIouThreshold,
             py::arg("threshold"),
             DOC(dai, beta, MPPalmDetectionParserConfig, setIouThreshold))
        .def("getIouThreshold", &beta::MPPalmDetectionParserConfig::getIouThreshold, DOC(dai, beta, MPPalmDetectionParserConfig, getIouThreshold))
        .def("setMaxDetections",
             &beta::MPPalmDetectionParserConfig::setMaxDetections,
             py::arg("maxDetections"),
             DOC(dai, beta, MPPalmDetectionParserConfig, setMaxDetections))
        .def("getMaxDetections", &beta::MPPalmDetectionParserConfig::getMaxDetections, DOC(dai, beta, MPPalmDetectionParserConfig, getMaxDetections))
        .def("validate", &beta::MPPalmDetectionParserConfig::validate, DOC(dai, beta, MPPalmDetectionParserConfig, validate));
}
