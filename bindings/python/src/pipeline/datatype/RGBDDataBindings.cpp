
#include <memory>

#include "DatatypeBindings.hpp"
#include "pipeline/CommonBindings.hpp"

// depthai
#include "depthai/pipeline/datatype/RGBDData.hpp"

// pybind
#include <pybind11/chrono.h>
#include <pybind11/numpy.h>

void bind_rgbddata(pybind11::module& m, void* pCallstack) {
    using namespace dai;

    py::class_<RGBDData, Py<RGBDData>, Buffer, std::shared_ptr<RGBDData>> rgbdData(m, "RGBDData", DOC(dai, RGBDData));

    ///////////////////////////////////////////////////////////////////////
    ///////////////////////////////////////////////////////////////////////
    ///////////////////////////////////////////////////////////////////////
    // Call the rest of the type defines, then perform the actual bindings
    Callstack* callstack = (Callstack*)pCallstack;
    auto cb = callstack->top();
    callstack->pop();
    cb(m, pCallstack);
    // Actual bindings
    ///////////////////////////////////////////////////////////////////////
    ///////////////////////////////////////////////////////////////////////
    ///////////////////////////////////////////////////////////////////////

    // Metadata / raw
    rgbdData.def(py::init<>())
        .def("__repr__", &RGBDData::str)
        .def("getRGBFrame", &RGBDData::getRGBFrame, DOC(dai, RGBDData, getRGBFrame))
        .def("getDepthFrame", &RGBDData::getDepthFrame, DOC(dai, RGBDData, getDepthFrame))
        .def("setRGBFrame", &RGBDData::setRGBFrame, py::arg("frame"), DOC(dai, RGBDData, setRGBFrame))
        .def("setDepthFrame", &RGBDData::setDepthFrame, py::arg("frame"), DOC(dai, RGBDData, setDepthFrame));
}
