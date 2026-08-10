#include "DatatypeBindings.hpp"
#include "depthai/beta/datatype/MLSDParserConfig.hpp"
#include "pipeline/CommonBindings.hpp"

void bind_beta_mlsdparserconfig(pybind11::module& m, void* pCallstack) {
    using namespace dai;
    auto betaModule = m.def_submodule("beta", "Experimental APIs");
    py::class_<beta::MLSDParserConfig, Py<beta::MLSDParserConfig>, Buffer, std::shared_ptr<beta::MLSDParserConfig>> config(
        betaModule, "MLSDParserConfig", DOC(dai, beta, MLSDParserConfig));

    Callstack* callstack = (Callstack*)pCallstack;
    auto cb = callstack->top();
    callstack->pop();
    cb(m, pCallstack);

    config.def(py::init<>())
        .def("__repr__", &beta::MLSDParserConfig::str)
        .def_readwrite("topK", &beta::MLSDParserConfig::topK)
        .def_readwrite("scoreThreshold", &beta::MLSDParserConfig::scoreThreshold)
        .def_readwrite("distanceThreshold", &beta::MLSDParserConfig::distanceThreshold)
        .def("setTopK", &beta::MLSDParserConfig::setTopK, py::arg("topK"), DOC(dai, beta, MLSDParserConfig, setTopK))
        .def("getTopK", &beta::MLSDParserConfig::getTopK, DOC(dai, beta, MLSDParserConfig, getTopK))
        .def("setScoreThreshold", &beta::MLSDParserConfig::setScoreThreshold, py::arg("threshold"), DOC(dai, beta, MLSDParserConfig, setScoreThreshold))
        .def("getScoreThreshold", &beta::MLSDParserConfig::getScoreThreshold, DOC(dai, beta, MLSDParserConfig, getScoreThreshold))
        .def(
            "setDistanceThreshold", &beta::MLSDParserConfig::setDistanceThreshold, py::arg("threshold"), DOC(dai, beta, MLSDParserConfig, setDistanceThreshold))
        .def("getDistanceThreshold", &beta::MLSDParserConfig::getDistanceThreshold, DOC(dai, beta, MLSDParserConfig, getDistanceThreshold))
        .def("validate", &beta::MLSDParserConfig::validate, DOC(dai, beta, MLSDParserConfig, validate));
}
