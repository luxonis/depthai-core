#include "DatatypeBindings.hpp"
#include "depthai/beta/datatype/SuperAnimalParserConfig.hpp"
#include "pipeline/CommonBindings.hpp"

void bind_beta_superanimalparserconfig(pybind11::module& m, void* pCallstack) {
    using namespace dai;
    auto betaModule = m.def_submodule("beta", "Experimental APIs");
    py::class_<beta::SuperAnimalParserConfig, Py<beta::SuperAnimalParserConfig>, Buffer, std::shared_ptr<beta::SuperAnimalParserConfig>> config(
        betaModule, "SuperAnimalParserConfig", DOC(dai, beta, SuperAnimalParserConfig));

    Callstack* callstack = (Callstack*)pCallstack;
    auto cb = callstack->top();
    callstack->pop();
    cb(m, pCallstack);

    config.def(py::init<>())
        .def("__repr__", &beta::SuperAnimalParserConfig::str)
        .def_readwrite("scoreThreshold", &beta::SuperAnimalParserConfig::scoreThreshold)
        .def("setScoreThreshold",
             &beta::SuperAnimalParserConfig::setScoreThreshold,
             py::arg("threshold"),
             DOC(dai, beta, SuperAnimalParserConfig, setScoreThreshold))
        .def("getScoreThreshold", &beta::SuperAnimalParserConfig::getScoreThreshold, DOC(dai, beta, SuperAnimalParserConfig, getScoreThreshold))
        .def("validate", &beta::SuperAnimalParserConfig::validate, DOC(dai, beta, SuperAnimalParserConfig, validate));
}
