#include "DatatypeBindings.hpp"
#include "depthai/beta/datatype/HRNetParserConfig.hpp"
#include "pipeline/CommonBindings.hpp"

void bind_beta_hrnetparserconfig(pybind11::module& m, void* pCallstack) {
    using namespace dai;
    auto betaModule = m.def_submodule("beta", "Experimental APIs");
    py::class_<beta::HRNetParserConfig, Py<beta::HRNetParserConfig>, Buffer, std::shared_ptr<beta::HRNetParserConfig>> config(
        betaModule, "HRNetParserConfig", DOC(dai, beta, HRNetParserConfig));

    Callstack* callstack = (Callstack*)pCallstack;
    auto cb = callstack->top();
    callstack->pop();
    cb(m, pCallstack);

    config.def(py::init<>())
        .def("__repr__", &beta::HRNetParserConfig::str)
        .def_readwrite("scoreThreshold", &beta::HRNetParserConfig::scoreThreshold)
        .def("setScoreThreshold", &beta::HRNetParserConfig::setScoreThreshold, py::arg("threshold"), DOC(dai, beta, HRNetParserConfig, setScoreThreshold))
        .def("getScoreThreshold", &beta::HRNetParserConfig::getScoreThreshold, DOC(dai, beta, HRNetParserConfig, getScoreThreshold))
        .def("validate", &beta::HRNetParserConfig::validate, DOC(dai, beta, HRNetParserConfig, validate));
}
