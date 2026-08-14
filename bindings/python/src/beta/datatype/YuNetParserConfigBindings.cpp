#include "DatatypeBindings.hpp"
#include "depthai/beta/datatype/YuNetParserConfig.hpp"
#include "pipeline/CommonBindings.hpp"

void bind_beta_yunetparserconfig(pybind11::module& m, void* pCallstack) {
    using namespace dai;
    auto betaModule = m.def_submodule("beta", "Experimental APIs");
    py::class_<beta::YuNetParserConfig, Py<beta::YuNetParserConfig>, Buffer, std::shared_ptr<beta::YuNetParserConfig>> config(
        betaModule, "YuNetParserConfig", DOC(dai, beta, YuNetParserConfig));

    Callstack* callstack = (Callstack*)pCallstack;
    auto cb = callstack->top();
    callstack->pop();
    cb(m, pCallstack);

    config.def(py::init<>())
        .def("__repr__", &beta::YuNetParserConfig::str)
        .def_readwrite("confidenceThreshold", &beta::YuNetParserConfig::confidenceThreshold)
        .def_readwrite("iouThreshold", &beta::YuNetParserConfig::iouThreshold)
        .def_readwrite("maxDetections", &beta::YuNetParserConfig::maxDetections)
        .def("setConfidenceThreshold",
             &beta::YuNetParserConfig::setConfidenceThreshold,
             py::arg("threshold"),
             DOC(dai, beta, YuNetParserConfig, setConfidenceThreshold))
        .def("getConfidenceThreshold", &beta::YuNetParserConfig::getConfidenceThreshold, DOC(dai, beta, YuNetParserConfig, getConfidenceThreshold))
        .def("setIouThreshold", &beta::YuNetParserConfig::setIouThreshold, py::arg("threshold"), DOC(dai, beta, YuNetParserConfig, setIouThreshold))
        .def("getIouThreshold", &beta::YuNetParserConfig::getIouThreshold, DOC(dai, beta, YuNetParserConfig, getIouThreshold))
        .def("setMaxDetections", &beta::YuNetParserConfig::setMaxDetections, py::arg("maxDetections"), DOC(dai, beta, YuNetParserConfig, setMaxDetections))
        .def("getMaxDetections", &beta::YuNetParserConfig::getMaxDetections, DOC(dai, beta, YuNetParserConfig, getMaxDetections))
        .def("validate", &beta::YuNetParserConfig::validate, DOC(dai, beta, YuNetParserConfig, validate));
}
