
#include "DatatypeBindings.hpp"
#include "pipeline/CommonBindings.hpp"
#include <unordered_map>
#include <memory>

// depthai
#include "depthai/pipeline/datatype/RGBDData.hpp"

//pybind
#include <pybind11/chrono.h>
#include <pybind11/numpy.h>

void bind_transformdata(pybind11::module& m, void* pCallstack){

    using namespace dai;

    py::class_<RGBDData, Py<RGBDData>, Buffer, std::shared_ptr<RGBDData>> rgbdData(m, "RGBDData", DOC(dai, RGBDData));

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

    // Metadata / raw
    rgbdData
        .def(py::init<>())
        .def("__repr__", &RGBDData::str)
        .def_readwrite("rgbFrame", &RGBDData::rgbFrame, DOC(dai, RGBDData, rgbFrame))
        .def_readwrite("depthFrame", &RGBDData::depthFrame, DOC(dai, RGBDData, depthFrame));
}
