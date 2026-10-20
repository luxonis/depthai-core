#include "DatatypeBindings.hpp"
#include "depthai/beta/datatype/SCRFDParserConfig.hpp"
#include "pipeline/CommonBindings.hpp"

void bind_beta_scrfdparserconfig(pybind11::module& m, void* pCallstack) {
    using namespace dai;
    auto betaModule = m.def_submodule("beta", "Experimental APIs");
    py::class_<beta::SCRFDParserConfig, Py<beta::SCRFDParserConfig>, Buffer, std::shared_ptr<beta::SCRFDParserConfig>> config(
        betaModule, "SCRFDParserConfig", DOC(dai, beta, SCRFDParserConfig));

    Callstack* callstack = (Callstack*)pCallstack;
    auto cb = callstack->top();
    callstack->pop();
    cb(m, pCallstack);

    config.def(py::init<>())
        .def("__repr__", &beta::SCRFDParserConfig::str)
        .def_readwrite("confidenceThreshold", &beta::SCRFDParserConfig::confidenceThreshold)
        .def_readwrite("iouThreshold", &beta::SCRFDParserConfig::iouThreshold)
        .def_readwrite("maxDetections", &beta::SCRFDParserConfig::maxDetections)
        .def("setConfidenceThreshold",
             &beta::SCRFDParserConfig::setConfidenceThreshold,
             py::arg("threshold"),
             DOC(dai, beta, SCRFDParserConfig, setConfidenceThreshold))
        .def("getConfidenceThreshold", &beta::SCRFDParserConfig::getConfidenceThreshold, DOC(dai, beta, SCRFDParserConfig, getConfidenceThreshold))
        .def("setIouThreshold", &beta::SCRFDParserConfig::setIouThreshold, py::arg("threshold"), DOC(dai, beta, SCRFDParserConfig, setIouThreshold))
        .def("getIouThreshold", &beta::SCRFDParserConfig::getIouThreshold, DOC(dai, beta, SCRFDParserConfig, getIouThreshold))
        .def("setMaxDetections", &beta::SCRFDParserConfig::setMaxDetections, py::arg("maxDetections"), DOC(dai, beta, SCRFDParserConfig, setMaxDetections))
        .def("getMaxDetections", &beta::SCRFDParserConfig::getMaxDetections, DOC(dai, beta, SCRFDParserConfig, getMaxDetections))
        .def("validate", &beta::SCRFDParserConfig::validate, DOC(dai, beta, SCRFDParserConfig, validate));
}
