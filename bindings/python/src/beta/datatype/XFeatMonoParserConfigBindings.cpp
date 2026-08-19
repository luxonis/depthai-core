#include "DatatypeBindings.hpp"
#include "depthai/beta/datatype/XFeatMonoParserConfig.hpp"
#include "pipeline/CommonBindings.hpp"

void bind_beta_xfeatmonoparserconfig(pybind11::module& m, void* pCallstack) {
    using namespace dai;
    auto betaModule = m.def_submodule("beta", "Experimental APIs");
    py::class_<beta::XFeatMonoParserConfig, Py<beta::XFeatMonoParserConfig>, Buffer, std::shared_ptr<beta::XFeatMonoParserConfig>> config(
        betaModule, "XFeatMonoParserConfig", DOC(dai, beta, XFeatMonoParserConfig));

    Callstack* callstack = (Callstack*)pCallstack;
    auto cb = callstack->top();
    callstack->pop();
    cb(m, pCallstack);

    config.def(py::init<>())
        .def("__repr__", &beta::XFeatMonoParserConfig::str)
        .def_readwrite("maxKeypoints", &beta::XFeatMonoParserConfig::maxKeypoints)
        .def("setMaxKeypoints", &beta::XFeatMonoParserConfig::setMaxKeypoints, py::arg("maxKeypoints"), DOC(dai, beta, XFeatMonoParserConfig, setMaxKeypoints))
        .def("getMaxKeypoints", &beta::XFeatMonoParserConfig::getMaxKeypoints, DOC(dai, beta, XFeatMonoParserConfig, getMaxKeypoints))
        .def("validate", &beta::XFeatMonoParserConfig::validate, DOC(dai, beta, XFeatMonoParserConfig, validate));
}
