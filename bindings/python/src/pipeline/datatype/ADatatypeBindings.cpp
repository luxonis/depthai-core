#include "DatatypeBindings.hpp"
#include "pipeline/CommonBindings.hpp"
#include <unordered_map>
#include <memory>

// depthai
#include "depthai/pipeline/datatype/ADatatype.hpp"

//pybind
#include <pybind11/chrono.h>
#include <pybind11/numpy.h>

// #include "spdlog/spdlog.h"

void bind_adatatype(pybind11::module& m, void* pCallstack){

    using namespace dai;

    py::class_<ADatatype, Py<ADatatype>, std::shared_ptr<ADatatype>> adatatype(m, "ADatatype", DOC(dai, ADatatype));

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

    adatatype.def(py::init<>(), DOC(dai, ADatatype, ADatatype));
    // Message
    // adatatype
        // .def("getRaw", &ADatatype::getRaw);

}
