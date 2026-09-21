#include "DatatypeBindings.hpp"
#include "depthai/beta/datatype/RFDETRParserConfig.hpp"
#include "pipeline/CommonBindings.hpp"

void bind_beta_rfdetrparserconfig(pybind11::module& m, void* pCallstack) {
    using namespace dai;
    auto betaModule = m.def_submodule("beta", "Experimental APIs");
    py::class_<beta::RFDETRParserConfig, Py<beta::RFDETRParserConfig>, Buffer, std::shared_ptr<beta::RFDETRParserConfig>> config(
        betaModule, "RFDETRParserConfig", DOC(dai, beta, RFDETRParserConfig));

    Callstack* callstack = (Callstack*)pCallstack;
    auto cb = callstack->top();
    callstack->pop();
    cb(m, pCallstack);

    config.def(py::init<>())
        .def("__repr__", &beta::RFDETRParserConfig::str)
        .def_readwrite("confidenceThreshold", &beta::RFDETRParserConfig::confidenceThreshold)
        .def_readwrite("maxDetections", &beta::RFDETRParserConfig::maxDetections)
        .def_readwrite("maskConfidence", &beta::RFDETRParserConfig::maskConfidence)
        .def("setConfidenceThreshold",
             &beta::RFDETRParserConfig::setConfidenceThreshold,
             py::arg("threshold"),
             DOC(dai, beta, RFDETRParserConfig, setConfidenceThreshold))
        .def("getConfidenceThreshold", &beta::RFDETRParserConfig::getConfidenceThreshold, DOC(dai, beta, RFDETRParserConfig, getConfidenceThreshold))
        .def("setMaxDetections", &beta::RFDETRParserConfig::setMaxDetections, py::arg("maxDetections"), DOC(dai, beta, RFDETRParserConfig, setMaxDetections))
        .def("getMaxDetections", &beta::RFDETRParserConfig::getMaxDetections, DOC(dai, beta, RFDETRParserConfig, getMaxDetections))
        .def("setMaskConfidence", &beta::RFDETRParserConfig::setMaskConfidence, py::arg("threshold"), DOC(dai, beta, RFDETRParserConfig, setMaskConfidence))
        .def("getMaskConfidence", &beta::RFDETRParserConfig::getMaskConfidence, DOC(dai, beta, RFDETRParserConfig, getMaskConfidence))
        .def("validate", &beta::RFDETRParserConfig::validate, DOC(dai, beta, RFDETRParserConfig, validate));
}
