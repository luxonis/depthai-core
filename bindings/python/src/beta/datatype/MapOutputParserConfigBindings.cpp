#include "DatatypeBindings.hpp"
#include "depthai/beta/datatype/MapOutputParserConfig.hpp"
#include "pipeline/CommonBindings.hpp"

void bind_beta_mapoutputparserconfig(pybind11::module& m, void* pCallstack) {
    using namespace dai;
    auto betaModule = m.def_submodule("beta", "Experimental APIs");
    py::class_<beta::MapOutputParserConfig, Py<beta::MapOutputParserConfig>, Buffer, std::shared_ptr<beta::MapOutputParserConfig>> config(
        betaModule, "MapOutputParserConfig", DOC(dai, beta, MapOutputParserConfig));

    Callstack* callstack = (Callstack*)pCallstack;
    auto cb = callstack->top();
    callstack->pop();
    cb(m, pCallstack);

    config.def(py::init<>())
        .def("__repr__", &beta::MapOutputParserConfig::str)
        .def_readwrite("minMaxScaling", &beta::MapOutputParserConfig::minMaxScaling)
        .def("setMinMaxScaling", &beta::MapOutputParserConfig::setMinMaxScaling, py::arg("enabled"), DOC(dai, beta, MapOutputParserConfig, setMinMaxScaling))
        .def("getMinMaxScaling", &beta::MapOutputParserConfig::getMinMaxScaling, DOC(dai, beta, MapOutputParserConfig, getMinMaxScaling))
        .def("validate", &beta::MapOutputParserConfig::validate, DOC(dai, beta, MapOutputParserConfig, validate));
}
