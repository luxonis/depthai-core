#include "DatatypeBindings.hpp"
#include "depthai/beta/datatype/PPTextDetectionParserConfig.hpp"
#include "pipeline/CommonBindings.hpp"

void bind_beta_pptextdetectionparserconfig(pybind11::module& m, void* pCallstack) {
    using namespace dai;
    auto betaModule = m.def_submodule("beta", "Experimental APIs");
    py::class_<beta::PPTextDetectionParserConfig, Py<beta::PPTextDetectionParserConfig>, Buffer, std::shared_ptr<beta::PPTextDetectionParserConfig>> config(
        betaModule, "PPTextDetectionParserConfig", DOC(dai, beta, PPTextDetectionParserConfig));

    Callstack* callstack = (Callstack*)pCallstack;
    auto cb = callstack->top();
    callstack->pop();
    cb(m, pCallstack);

    config.def(py::init<>())
        .def("__repr__", &beta::PPTextDetectionParserConfig::str)
        .def_readwrite("confidenceThreshold", &beta::PPTextDetectionParserConfig::confidenceThreshold)
        .def_readwrite("maskThreshold", &beta::PPTextDetectionParserConfig::maskThreshold)
        .def_readwrite("maxDetections", &beta::PPTextDetectionParserConfig::maxDetections)
        .def("setConfidenceThreshold",
             &beta::PPTextDetectionParserConfig::setConfidenceThreshold,
             py::arg("threshold"),
             DOC(dai, beta, PPTextDetectionParserConfig, setConfidenceThreshold))
        .def("getConfidenceThreshold",
             &beta::PPTextDetectionParserConfig::getConfidenceThreshold,
             DOC(dai, beta, PPTextDetectionParserConfig, getConfidenceThreshold))
        .def("setMaskThreshold",
             &beta::PPTextDetectionParserConfig::setMaskThreshold,
             py::arg("threshold"),
             DOC(dai, beta, PPTextDetectionParserConfig, setMaskThreshold))
        .def("getMaskThreshold", &beta::PPTextDetectionParserConfig::getMaskThreshold, DOC(dai, beta, PPTextDetectionParserConfig, getMaskThreshold))
        .def("setMaxDetections",
             &beta::PPTextDetectionParserConfig::setMaxDetections,
             py::arg("maxDetections"),
             DOC(dai, beta, PPTextDetectionParserConfig, setMaxDetections))
        .def("getMaxDetections", &beta::PPTextDetectionParserConfig::getMaxDetections, DOC(dai, beta, PPTextDetectionParserConfig, getMaxDetections))
        .def("validate", &beta::PPTextDetectionParserConfig::validate, DOC(dai, beta, PPTextDetectionParserConfig, validate));
}
