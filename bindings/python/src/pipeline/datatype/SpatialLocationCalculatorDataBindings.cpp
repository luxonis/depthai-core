#include <memory>
#include <unordered_map>

#include "DatatypeBindings.hpp"
#include "pipeline/CommonBindings.hpp"

#include "depthai/pipeline/datatype/SpatialLocationCalculatorData.hpp"

#include <pybind11/chrono.h>
#include <pybind11/numpy.h>

void bind_spatiallocationcalculatordata(pybind11::module& m, void* pCallstack) {
    using namespace dai;

    py::class_<SpatialLocationCalculatorData, Py<SpatialLocationCalculatorData>, Buffer, py::smart_holder>
        spatialLocationCalculatorData(m, "SpatialLocationCalculatorData", DOC(dai, SpatialLocationCalculatorData));
    py::class_<SpatialLocations> spatialLocations(m, "SpatialLocations", DOC(dai, SpatialLocations));

    Callstack* callstack = (Callstack*)pCallstack;
    auto cb = callstack->top();
    callstack->pop();
    cb(m, pCallstack);

    spatialLocations.def(py::init<>())
        .def_readwrite("config", &SpatialLocations::config, DOC(dai, SpatialLocations, config))
        .def_readwrite("depthAverage", &SpatialLocations::depthAverage, DOC(dai, SpatialLocations, depthAverage))
        .def_readwrite("depthMode", &SpatialLocations::depthMode, DOC(dai, SpatialLocations, depthMode))
        .def_readwrite("depthMedian", &SpatialLocations::depthMedian, DOC(dai, SpatialLocations, depthMedian))
        .def_readwrite("depthMin", &SpatialLocations::depthMin, DOC(dai, SpatialLocations, depthMin))
        .def_readwrite("depthMax", &SpatialLocations::depthMax, DOC(dai, SpatialLocations, depthMax))
        .def_readwrite("depthAveragePixelCount", &SpatialLocations::depthAveragePixelCount, DOC(dai, SpatialLocations, depthAveragePixelCount))
        .def_readwrite("spatialCoordinates", &SpatialLocations::spatialCoordinates, DOC(dai, SpatialLocations, spatialCoordinates));

    spatialLocationCalculatorData.def(py::init<>())
        .def("__repr__", &SpatialLocationCalculatorData::str)
        .def("getSpatialLocations", &SpatialLocationCalculatorData::getSpatialLocations, DOC(dai, SpatialLocationCalculatorData, getSpatialLocations))
        .def_property(
            "spatialLocations",
            [](SpatialLocationCalculatorData& loc) -> std::vector<SpatialLocations>& { return loc.spatialLocations; },
            [](SpatialLocationCalculatorData& loc, std::vector<SpatialLocations>& val) { loc.spatialLocations = val; },
            DOC(dai, SpatialLocationCalculatorData, spatialLocations));
}
