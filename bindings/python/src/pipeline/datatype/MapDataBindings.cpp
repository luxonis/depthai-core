#include <memory>
#include <unordered_map>

#include "DatatypeBindings.hpp"
#include "pipeline/CommonBindings.hpp"

// depthai
#include "depthai/pipeline/datatype/MapData.hpp"

// pybind
#include <pybind11/chrono.h>
#include <pybind11/numpy.h>

void bind_mapdata(pybind11::module& m, void* pCallstack) {
    using namespace dai;

    py::class_<MapData, Py<MapData>, Buffer, std::shared_ptr<MapData>> mapData(m, "MapData", DOC(dai, MapData));

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
    mapData.def(py::init<>())
        .def("__repr__", &MapData::str)
        .def_readwrite("map", &MapData::map)
        .def_readwrite("minX", &MapData::minX)
        .def_readwrite("minY", &MapData::minY);
}
