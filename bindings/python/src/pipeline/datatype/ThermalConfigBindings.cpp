#include "DatatypeBindings.hpp"
#include "pipeline/CommonBindings.hpp"

// depthai
#include "depthai/pipeline/datatype/ThermalConfig.hpp"

// #include "spdlog/spdlog.h"

void bind_thermalconfig(pybind11::module& m, void* pCallstack){

    using namespace dai;

    py::class_<ThermalConfig, Py<ThermalConfig>, Buffer, std::shared_ptr<ThermalConfig>> thermalConfig(m, "ThermalConfig", DOC(dai, ThermalConfig));

    ///////////////////////////////////////////////////////////////////////
    ///////////////////////////////////////////////////////////////////////
    ///////////////////////////////////////////////////////////////////////
    // Call the rest of the type defines, then perform the actual bindings
    Callstack* callstack = (Callstack*) pCallstack;
    auto cb = callstack->top();
    callstack->pop();
    cb(m, pCallstack);
    // Actual bindings
    ///////////////////////////////////////////////////////////////////////
    ///////////////////////////////////////////////////////////////////////
    ///////////////////////////////////////////////////////////////////////

    // Message
    thermalConfig
        .def(py::init<>())
        .def_readwrite("autoFFC", &ThermalConfig::autoFFC, DOC(dai, ThermalConfig, autoFFC))
    ;

}
