#include "DatatypeBindings.hpp"
#include "depthai/beta/datatype/XFeatStereoParserConfig.hpp"
#include "pipeline/CommonBindings.hpp"

void bind_beta_xfeatstereoparserconfig(pybind11::module& m, void* pCallstack) {
    using namespace dai;
    auto betaModule = m.def_submodule("beta", "Experimental APIs");
    py::class_<beta::XFeatStereoParserConfig, Py<beta::XFeatStereoParserConfig>, Buffer, std::shared_ptr<beta::XFeatStereoParserConfig>> config(
        betaModule, "XFeatStereoParserConfig", DOC(dai, beta, XFeatStereoParserConfig));

    Callstack* callstack = (Callstack*)pCallstack;
    auto cb = callstack->top();
    callstack->pop();
    cb(m, pCallstack);

    config.def(py::init<>())
        .def("__repr__", &beta::XFeatStereoParserConfig::str)
        .def_readwrite("maxKeypoints", &beta::XFeatStereoParserConfig::maxKeypoints)
        .def("setMaxKeypoints",
             &beta::XFeatStereoParserConfig::setMaxKeypoints,
             py::arg("maxKeypoints"),
             DOC(dai, beta, XFeatStereoParserConfig, setMaxKeypoints))
        .def("getMaxKeypoints", &beta::XFeatStereoParserConfig::getMaxKeypoints, DOC(dai, beta, XFeatStereoParserConfig, getMaxKeypoints))
        .def("validate", &beta::XFeatStereoParserConfig::validate, DOC(dai, beta, XFeatStereoParserConfig, validate));
}
